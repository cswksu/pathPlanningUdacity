#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
#ifdef UWS_VCPKG // for windows install
  string map_file_ = "./data/highway_map.csv";
#else // for unix/mac
  string map_file_ = "../data/highway_map.csv";
#endif
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  vector<double> prevX = {0}; //persistent vectors for previous X, Y, acc
  vector<double> prevY = { 0 };
  vector<double> prevAcc = { 0 };
  vector<double> yRate = { 0 };
  int targLane = 1;
  bool commitChangeL = false; //persistent vectors for committing to lane change
  bool commitChangeR = false;
  

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
#ifdef UWS_VCPKG  //windows
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
    &map_waypoints_dx, &map_waypoints_dy, &commitChangeL, &commitChangeR, &prevX, &prevY, &prevAcc, &yRate, &targLane]
    (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
      uWS::OpCode opCode) {
#else //unix/mac
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
    &map_waypoints_dx, &map_waypoints_dy, &commitChangeL, &commitChangeR, &prevX, &prevY, &prevAcc, &yRate, &targLane]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
#endif
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"]; //in mph
          double maxDistTravel=0.44704; // distance in meters to travel in 1 step
          double ref_speed = 45.0 * maxDistTravel; //45 mph
          double max_speed = ref_speed+2.0*maxDistTravel; //47 mph
          double min_speed = ref_speed-2.0*maxDistTravel; //43 mph

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          int prevPathSize=previous_path_x.size(); //points in previous trajectory
          double pos_x; //latest x position
          double pos_y; //latest y position
          double pos_s; //longitudinal lane position
          double pos_d; //lateral lane position
          double prev_pos_x; //second to latest x
          double prev_pos_y;
          double prev_prev_pos_x; //third to latest x
          double prev_prev_pos_y;
          double theta; //theta in radians
          double speed=0; //speed in m/s
          double acc_tan=0; //tangential acceleration in m/s^2
          double lastSpeed = 0; //second to last speed
          double ts = 0.02; //timestep
          const double max_dist = 6945.554/2.0; //furthest s-distance possible
          double yaw_rate = 0;

          prevPathSize=std::min(prevPathSize,15); //cap previous trajectory at 15 steps (0.3s)
          for (int i =0; i < prevPathSize; ++i) { //prefill trajectory with retained points
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          if (prevPathSize==0) { //if no path to continue from
            pos_x = car_x;
            pos_y = car_y;
            theta=deg2rad(car_yaw);
            pos_s=car_s;
            pos_d=car_d;
            speed = car_speed * 0.44704;
            pos_s = car_s;
            pos_d = car_d;
          } else if (prevPathSize == 1) { //if path has only 1 point
            speed = car_speed* 0.44704;
            theta = deg2rad(car_yaw);
            pos_x = previous_path_x[0];
            pos_y = previous_path_y[0];
            pos_s = car_s;
            pos_d = car_d;
          } else if (prevPathSize == 2) { //if path has 2 points
            pos_x= previous_path_x[prevPathSize - 1];
            pos_y = previous_path_y[prevPathSize - 1];
            prev_pos_x = previous_path_x[prevPathSize - 2];
            prev_pos_y = previous_path_y[prevPathSize - 2];
            //get kinematics of situation
            vector<double> kine2p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, ts);
            speed = kine2p[0];
            theta = kine2p[1];
            pos_s = car_s;
            pos_d = car_d;
          } else { //greater than 2 poitns
            pos_x = previous_path_x[prevPathSize - 1];
            pos_y = previous_path_y[prevPathSize - 1];
            prev_pos_x = previous_path_x[prevPathSize - 2];
            prev_pos_y = previous_path_y[prevPathSize - 2];
            prev_prev_pos_x = previous_path_x[prevPathSize - 3];
            prev_prev_pos_y = previous_path_y[prevPathSize - 3];
            vector<double> kine3p = kinematics(next_x_vals, next_y_vals, ts); //get kinematics
            speed = kine3p[0];
            theta = kine3p[1];
            lastSpeed = kine3p[2];
            acc_tan = kine3p[3];


            if (prevPathSize == 15) {
              vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
              pos_s = frenetPos[0]; //calculate frenet position
              pos_d = frenetPos[1];
            } else {
              pos_s = car_s; //frenet position already calculated
              pos_d = car_d;
            }
          }
          int lenPrev = prevX.size();  //get length of previous trajectory
          
          //search previous trajectory for match to pos_x, pos_y
          for (int i = 0; i < lenPrev; ++i) {
            if ((abs(pos_x - prevX[i]) < 0.01) && (abs(pos_y - prevY[i]) < 0.01)) {
              acc_tan = prevAcc[i]; //match to more accurate tangential acceleration
              yaw_rate = yRate[i];
              break;
            }
          }
            
          int lane = round((pos_d - 2.0) / 4.0); //calculate current lane
          
          
          //initialize values for vehicles surrounding ego vehicle
          double lane1AheadDist = 999;
          double lane2AheadDist = 999;
          double lane3AheadDist = 999;
          double lane1BehindDist = 999;
          double lane2BehindDist = 999;
          double lane3BehindDist = 999;
          double lane1AheadSpd = 999;
          double lane2AheadSpd = 999;
          double lane3AheadSpd = 999;
          double lane1BehindSpd = 0;
          double lane2BehindSpd = 0;
          double lane3BehindSpd = 0;
          double lane1AheadTTC = 999;
          double lane2AheadTTC = 999;
          double lane3AheadTTC = 999;
          double lane1BehindTTC = 999;
          double lane2BehindTTC = 999;
          double lane3BehindTTC = 999;

          //values from individual fusion measurement
          double targetX_speed;
          double sf_s;
          double future_s;
          double sf_d;
          double sf_vx;
          double sf_vy;
          
          for (int i=0; i<sensor_fusion.size(); ++i) {
            //read in from sensor fusion entry
            sf_d=sensor_fusion[i][6];
            sf_s = sensor_fusion[i][5];
            sf_vx = sensor_fusion[i][3];
            sf_vy = sensor_fusion[i][4];
            targetX_speed = sqrt(pow(sf_vx, 2) + pow(sf_vy, 2));
            future_s = sf_s + ts * prevPathSize*targetX_speed; //project from time 0 to now
            
            //check if distance is greater than half of track length
            while (abs(future_s - pos_s) > max_dist) {
              //calculate actual distance
              if (future_s > pos_s) {
                future_s -= max_dist * 2;
              }
              else {
                future_s += max_dist * 2;
              }
            }
            if (sf_d<4) { //lane 1
              if (future_s > pos_s) {
                if ((future_s - pos_s) < lane1AheadDist) {
                  lane1AheadDist = future_s - pos_s;
                  lane1AheadSpd = targetX_speed;
                  lane1AheadTTC = lane1AheadDist / (speed - targetX_speed);
                  if (lane1AheadTTC < 0) {
                    lane1AheadTTC = 999;
                  }
                }
              }
              else {
                if ((pos_s - future_s) < lane1BehindDist) {
                  lane1BehindDist = pos_s - future_s;
                  lane1BehindSpd = targetX_speed;
                  lane1BehindTTC = lane1BehindDist / (-speed + targetX_speed);
                  if (lane1BehindTTC < 0) {
                    lane1BehindTTC = 999;
                  }
                }
              }
            }
            else if (sf_d < 8) { //lane 2
              if (future_s > pos_s) {
                if ((future_s - pos_s) < lane2AheadDist) {
                  lane2AheadDist = future_s - pos_s;
                  lane2AheadSpd = targetX_speed;
                  lane2AheadTTC = lane2AheadDist / (speed - targetX_speed);
                  if (lane2AheadTTC < 0) {
                    lane2AheadTTC = 999;
                  }
                }
                
              }
              else {
                if ((pos_s - future_s) < lane2BehindDist) {
                  lane2BehindDist = pos_s - future_s;
                  lane2BehindSpd = targetX_speed;
                  lane2BehindTTC = lane2BehindDist / (-speed + targetX_speed);
                  if (lane2BehindTTC < 0) {
                    lane2BehindTTC = 999;
                  }
                }
              }
            }
            else if (sf_d>=8) { //lane 3
              if (future_s > pos_s) {
                if ((future_s - pos_s) < lane3AheadDist) {
                  lane3AheadDist = future_s - pos_s;
                  lane3AheadSpd = targetX_speed;
                  lane3AheadTTC = lane3AheadDist / (speed - targetX_speed);
                  if (lane3AheadTTC < 0) {
                    lane3AheadTTC = 999;
                  }
                }
              }
              else {
              if ((pos_s - future_s) < lane3BehindDist) {
                lane3BehindDist = pos_s - future_s;
                lane3BehindSpd = targetX_speed;
                lane3BehindTTC = lane3BehindDist / (-speed + targetX_speed);
                if (lane3BehindTTC < 0) {
                  lane3BehindTTC = 999;
                }
              }
              }
            }
          }
          
          double car_ahead_dist = 999;
          double car_ahead_speed = 999;
          //feasibility of left and right lane change
          bool changeRightFeas = false;
          bool changeLeftFeas = false;
          if (lane == 0) { //host lane is 1
            car_ahead_dist = lane1AheadDist;
            car_ahead_speed = lane1AheadSpd;
            //check for lane 2 feasibility
            if ((lane2BehindTTC > 3.0) && (lane2BehindDist > 15) && (lane2AheadTTC > lane1AheadTTC) && (lane2AheadDist > 15) && (lane1AheadDist > 15) && (lane2AheadSpd > lane1AheadSpd) && (speed > 10)) {
              changeRightFeas = true;
            }
          }
          else if (lane == 1) { //host lane is 2
            car_ahead_dist = lane2AheadDist;
            car_ahead_speed = lane2AheadSpd;
            //check for lane 3 feasibility
            if ((lane3BehindTTC> 3.0) && (lane3BehindDist>15) && (lane3AheadTTC > lane2AheadTTC) && (lane3AheadDist > 15) && (lane2AheadDist > 15) && (lane3AheadSpd > lane2AheadSpd) && (speed > 10)) {
              changeRightFeas = true;
            }
            //check for lane 1 feasibility
            if ((lane1BehindTTC > 3.0) && (lane1BehindDist > 15) && (lane1AheadTTC > lane2AheadTTC) && (lane2AheadDist>15) && (lane1AheadDist>15) && (lane2AheadSpd > lane1AheadSpd) && (speed > 10)) {
              changeLeftFeas = true;
            }

            if (changeLeftFeas && changeRightFeas) {
              if (lane1AheadSpd > lane3AheadSpd) {
                changeRightFeas = false;
              }
              else {
                changeLeftFeas = false;
              }
            }
          }
          else if (lane > 1) { //host lane is 3
            car_ahead_dist = lane3AheadDist;
            car_ahead_speed = lane3AheadSpd;
            //check for lane 2 feasibility
            if ((lane2BehindTTC > 3.0) && (lane2BehindDist > 15) && (lane2AheadTTC > lane3AheadTTC) && (lane2AheadDist > 15) && (lane3AheadDist > 15) && (lane2AheadSpd > lane3AheadSpd) && (speed > 10)) {
              changeLeftFeas = true;
            }
          }
          if ((abs(acc_tan) > 1.0) || (abs(yaw_rate)>0.1)) { //prohibit lane changes while changing speeds
            changeLeftFeas = false;
            changeRightFeas = false;
          }
          if ((changeLeftFeas) && (!commitChangeR)&&(!commitChangeL)) { //if lane change left is feasible
            if (lane > 0) {
              targLane=lane-1; //lane change
              commitChangeL = true; //commit to left lane change
              std::cout << "Left lane change" << std::endl;
            }

          }
          else if ((changeRightFeas) && (!commitChangeL) && (!commitChangeR)) { //if lane change right is feasible
            if (lane < 2) {
              targLane=lane+1; //lane change
              commitChangeR = true; //commit to change
              std::cout << "Right lane change" << std::endl;
            }

          }
          /*else if (commitChangeL) { //if ego vehicle has committed to change left
            if (lane > 0) {
              --lane;
            }
          }
          else if (commitChangeR) { //if ego vehicle has committed to change right
            if (lane < 2) {
              ++lane;              
            }
          }*/
          if ((pos_d > 3.5) && (pos_d < 4.5)) {
            car_ahead_speed=std::min(lane1AheadSpd,lane2AheadSpd);
          }
          else if ((pos_d > 7.5) && (pos_d < 8.5)) {
            car_ahead_speed = std::min(lane3AheadSpd, lane2AheadSpd);
          }

          if (car_ahead_speed < ref_speed) { //if car ahead is slower than desired speed
            double ttc =  car_ahead_dist / (2*ref_speed -car_ahead_speed); //calculate time to collision
            if (ttc < 3.0) {
              ref_speed = std::max(car_ahead_speed,0.0); //follow at speed
            }
            if (ttc < 2.0) {
              ref_speed = std::max(car_ahead_speed - 2.5*maxDistTravel,0.0); //slow down slightly
            }
            

          }
          else {
            ref_speed = 45.0 * maxDistTravel; //travel at 45 mph

          }
          
          max_speed = ref_speed + 2.0*maxDistTravel; //2mph over reference speed
          min_speed = ref_speed - 2.0*maxDistTravel; //2mph under
          
          int projSteps = 5; //5 spline control points
          int targSteps = 100; //target 2 s of motion
          vector<double> xPath(projSteps), yPath(projSteps);
          double xOffset = pos_x; //recenter transformation to pos_x, pos_y
          double yOffset = pos_y;
          xPath[0] = 0;
          yPath[0] = 0;
          double shift = targLane * 4.0 + 2.0 - pos_d; //required change in pos_d
          double lastS = pos_s;
          double lastD = pos_d;
          for (int i = 1; i < projSteps; ++i) {
            double stage = 1.0 / (projSteps - i); //how far in lane change
            double tempS = pos_s + i * (speed + 5)*targSteps*stage*ts; //s control point
            double deltaS = tempS - lastS;
            double tempD = pos_d + shift; //d control point
            if ((commitChangeL) || (commitChangeR)) { //if committed to change lanes
              if (commitChangeL) {
                //tempD = lastD - deltaS * param*std::min(0.4, exp(progress * 10)*pow(exp(progress * 10) + 1, -2)) /targSteps;
                tempD = lastD - 4.0 * i / projSteps;
                if (tempD < (targLane * 4 + 2.0)) {
                  tempD = targLane * 4 + 2.0;
                }
              }
              else {
                //tempD = lastD + deltaS * param*std::min(0.4,exp(progress*10)*pow(exp(progress*10) + 1, -2))/targSteps;
                tempD = lastD + 4.0 * i / projSteps;
                if (tempD > (targLane * 4 + 2.0)) {
                  tempD = targLane * 4 + 2.0;
                }

              }
              
              
            }
            vector<double> xyTemp= getXY(tempS, tempD,map_waypoints_s, map_waypoints_x, map_waypoints_y);
            xPath[i]=xyTemp[0]-xOffset; //add to spline
            yPath[i]=xyTemp[1]-yOffset;
            lastD = tempD;
            lastS = tempS;
            
          }
          double thetaRotCW=atan2(yPath[projSteps-1],xPath[projSteps-1]); //rotation angle
          vector<double> xTransPath(projSteps), yTransPath(projSteps); //transformed spline path
          xTransPath[0] = 0;
          yTransPath[0] = 0;
          for (int i=1; i < projSteps; ++i) {
            vector<double> cwRot = rotateCW(xPath[i], yPath[i], thetaRotCW);
            //transform so that spline is along x-axis
            xTransPath[i] = cwRot[0];
            yTransPath[i]= cwRot[1];              
          }
          
          double pos_x_trans=0;
          double pos_y_trans=0;
          tk::spline s;
          s.set_points(xTransPath,yTransPath); //calculate spline
          prevX.clear(); //clear persistent x, y, acc_tan vectors
          prevY.clear();
          prevAcc.clear();
          yRate.clear();
          for (int i =0; i < 50 - prevPathSize; ++i) {
            bool underspeed = (speed <= min_speed);
            bool overspeed = (speed >= max_speed);
            bool overAcc = (acc_tan > 3.0);
            bool coastDown = (acc_tan >= sqrt(14.0*(abs(speed-ref_speed)))); //reduce jerk when releasing throttle
            bool coastUp = (-acc_tan >= sqrt(6.0 * abs(speed - ref_speed))); //slowly release brakes
            if (underspeed) {
              if ((!overAcc) && (!coastDown)) {
                acc_tan = std::min(3.0, acc_tan + ts*7.0); //cap acc at 3 m/s^2, jerk at 7 m/s^3
              }
              
              if (overAcc) {
                acc_tan = 3.0; //cap acc at 3 m/s^2
              }

              if (coastDown) {
                acc_tan =std::max(-1.0, acc_tan-7.0 * ts);
                //come off throttle, max decel -1 m/s^2, jerk of -7 m/s^3
              }
            } else if (overspeed ) {
              if (overAcc) {
                acc_tan = 3.0;
              }
              
              if (coastUp) {
                acc_tan = std::min(1.0, acc_tan + 3.0*ts);
                //come off brakes at 3 m/s^3, max accel 1 m/s^2
              }
              else {
                acc_tan = std::max(-7.0, acc_tan - 8.0*ts);
                //brake at -8 m/s^3 jerk, max -7 m/s^2
              }
            } else {
              if (coastDown) {
                acc_tan = std::max(-1.0, acc_tan - 7.0 * ts);
              } else if (coastUp) {
                  acc_tan = std::min(1.0, acc_tan + 3.0*ts);
              } else if (abs(acc_tan) < 1.5 * ts) {
                acc_tan = 0;
                //continue at speed
              }
            }
            //double check that accelerations are within bounds
            acc_tan = std::max(acc_tan, -7.0);
            acc_tan = std::min(acc_tan, 3.0);
            speed += acc_tan * ts; //increment acceleration
            double allowableDiff=0.03; //0.03 m/s difference allowable from plan
            double tempX=pos_x_trans+speed*ts; //initial trajectory guess
            double tempY=s(tempX);
            //speed of initial trajectory guess
            double tempSpeed=speedCalc(tempX, tempY, pos_x_trans,pos_y_trans,ts);
            bool lastOverspeed = true;
            double incrementer = 0.01;
            //while trajectory results in unsatisfactory speed, generate new trajectory
            while(abs(tempSpeed-speed)>allowableDiff) {
              if (tempSpeed-speed>0) {
                if (!lastOverspeed) {
                  incrementer *= 0.5;
                }

                tempX -= incrementer;
                lastOverspeed = true;
              } else {
                if (lastOverspeed) {
                  incrementer *= 0.5;
                  
                }
                tempX += incrementer;
                lastOverspeed = false;
                
              }
              tempY=s(tempX);
              tempSpeed = speedCalc(tempX, tempY, pos_x_trans, pos_y_trans, ts);
              


            }
            double overageFactor = speed / tempSpeed; //correction factor
            double deltaX = (tempX-pos_x_trans)*overageFactor;
            double deltaY = (tempY-pos_y_trans)*overageFactor;
            prev_pos_x=pos_x;
            prev_pos_y=pos_y;
            double xPostDelta = deltaX + pos_x_trans; //coordinates in transformed system
            double yPostDelta = deltaY + pos_y_trans;
            
            //transform back to real coordinate system
            vector<double> rotCCW = rotateCCW(xPostDelta, yPostDelta, thetaRotCW);
            pos_x = rotCCW[0]+xOffset;
            pos_y= rotCCW[1]+yOffset;
            pos_x_trans+=deltaX;
            pos_y_trans+=deltaY;
            next_x_vals.push_back(pos_x); //push into trajectory output
            next_y_vals.push_back(pos_y);
            if (next_x_vals.size() < 3) { //if only 1 or 2 points in trajectory
              vector<double> kine2p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, ts);
              
              //get kinematics
              speed = kine2p[0];
              yaw_rate = (kine2p[1] - theta) / ts;

              theta = kine2p[1];
              
              lastSpeed = speed-acc_tan*ts;
            } else {
              prev_prev_pos_x=next_x_vals[next_x_vals.size()-3];
              prev_prev_pos_y=next_y_vals[next_y_vals.size()-3];
              vector<double> kine3p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, prev_prev_pos_x, prev_prev_pos_y, ts);
              //get kinematics
              speed = kine3p[0];
              yaw_rate = (kine3p[1] - theta) / ts;
              theta = kine3p[1];
              lastSpeed = kine3p[2];
              acc_tan = kine3p[3];
            }
            vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
            pos_s = frenetPos[0];
            pos_d = frenetPos[1];

            if ((commitChangeL||commitChangeR)&&(abs(pos_d - targLane * 4 - 2.0) < 0.2)) {
              commitChangeL = false; //completion of lane change
              commitChangeR = false;
              std::cout << "lane change complete" << std::endl;
            }
            
            yRate.push_back(yaw_rate);
            prevX.push_back(pos_x); //push trajectory and acceleration into persistent storage
            prevY.push_back(pos_y);
            prevAcc.push_back(acc_tan);
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
#ifdef UWS_VCPKG //windows only
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG //windows only
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }  // end websocket if
  }); // end h.onMessage


#ifdef UWS_VCPKG //windows only
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
#endif

#ifdef UWS_VCPKG //windows only
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
    char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
    char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
#endif

  int port = 4567;
#ifdef UWS_VCPKG //windows only
  if (h.listen("127.0.0.1", port)) {
#else 
  if (h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

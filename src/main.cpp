#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;
//using Eigen::MatrixXd;
//using Eigen::VectorXd;


int main() {
  uWS::Hub h;
  //typedef std::numeric_limits< double > dbl;
  //std::cout.precision(dbl::max_digits10);

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  //char state;
  //a for accelerate in lane
  //c for constrained by ahead vehicle
  //k for prepare lane change left
  //l for lane change left
  //s for prepare lane change right
  //r for lane change right

  // Waypoint map to read from
#ifdef UWS_VCPKG
  string map_file_ = "./data/highway_map.csv";
#else
  string map_file_ = "../data/highway_map.csv";
#endif
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  vector<double> prevX = {0};
  vector<double> prevY = { 0 };
  vector<double> prevAcc = { 0 };
  bool commitChangeL = false;
  bool commitChangeR = false;
  

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  //if (in_map_.good()) {
    //std::cout << "file exists" << std::endl;
  //}
  //std::cout << "reading in lines" << std::endl;
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
    //std::cout << "read in a line" << std::endl;
  }
  //std::cout << "read in all lines" << std::endl;
#ifdef UWS_VCPKG
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
    &map_waypoints_dx, &map_waypoints_dy, &commitChangeL, &commitChangeR, &prevX, &prevY, &prevAcc]
    (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
      uWS::OpCode opCode) {
#else
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
    &map_waypoints_dx, &map_waypoints_dy, &commitChangeL, &commitChangeR, &prevX, &prevY, &prevAcc]
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
          std::cout<<"*******msg received*******"<<std::endl;
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          //car_speed *= 0.44704;
          double maxDistTravel=0.44704; // distance in meters to travel
          double ref_speed = 45.0 * maxDistTravel;
          double max_speed = ref_speed+2.0*maxDistTravel;
          double min_speed = ref_speed-2.0*maxDistTravel;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          //double end_path_s = j[1]["end_path_s"];
          //double end_path_d = j[1]["end_path_d"];

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

          
          
          //std::cout << "received data" << std::endl;
          int prevPathSize=previous_path_x.size();
          double pos_x;
          double pos_y;
          double pos_s;
          double pos_d;
          double prev_pos_x;
          double prev_pos_y;
          double prev_prev_pos_x;
          double prev_prev_pos_y;
          double theta;
          double speed=0;
          double acc_tan=0;
          double lastSpeed = 0;
          double ts = 0.02;
          const double max_dist = 6945.554/2.0;

          prevPathSize=std::min(prevPathSize,15);
          std::cout << "Previous path size: " << prevPathSize << std::endl;
          for (int i =0; i < prevPathSize; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          if (prevPathSize==0) {
            pos_x = car_x;
            pos_y = car_y;
            std::cout << "x: " << pos_x << " y: " << pos_y << std::endl;
            theta=deg2rad(car_yaw);
            pos_s=car_s;
            pos_d=car_d;
            speed = car_speed * 0.44704;
            pos_s = car_s;
            pos_d = car_d;
          } else if (prevPathSize == 1) {
            speed = car_speed* 0.44704;
            theta = deg2rad(car_yaw);
            pos_x = previous_path_x[0];
            pos_y = previous_path_y[0];
            std::cout << "x: " << pos_x << " y: " << pos_y << std::endl;
            pos_s = car_s;
            pos_d = car_d;
          } else if (prevPathSize == 2) {
            pos_x= previous_path_x[prevPathSize - 1];
            pos_y = previous_path_y[prevPathSize - 1];
            std::cout << "x: " << pos_x << " y: " << pos_y << std::endl;
            prev_pos_x = previous_path_x[prevPathSize - 2];
            prev_pos_y = previous_path_y[prevPathSize - 2];
            vector<double> kine2p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, ts);
            speed = kine2p[0];
            theta = kine2p[1];
            pos_s = car_s;
            pos_d = car_d;
          } else {
            pos_x = previous_path_x[prevPathSize - 1];
            pos_y = previous_path_y[prevPathSize - 1];
            std::cout << "x: " << pos_x << " y: " << pos_y << std::endl;
            prev_pos_x = previous_path_x[prevPathSize - 2];
            prev_pos_y = previous_path_y[prevPathSize - 2];
            std::cout << "x_-1: " << prev_pos_x << " y_-1: " << prev_pos_y << std::endl;
            prev_prev_pos_x = previous_path_x[prevPathSize - 3];
            prev_prev_pos_y = previous_path_y[prevPathSize - 3];
            std::cout << "x_-2: " << prev_prev_pos_x << " y_-2: " << prev_prev_pos_y << std::endl;
            vector<double> kine3p = kinematics(next_x_vals, next_y_vals, ts);
            speed = kine3p[0];
            theta = kine3p[1];
            lastSpeed = kine3p[2];
            acc_tan = kine3p[3];


            if (prevPathSize == 15) {
              vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
              pos_s = frenetPos[0];
              pos_d = frenetPos[1];
            } else {
              pos_s = car_s;
              pos_d = car_d;
            }
          }
          int lenPrev = prevX.size();
          for (int i = 0; i < lenPrev; ++i) {
            if ((abs(pos_x - prevX[i]) < 0.01) && (abs(pos_y - prevY[i]) < 0.01)) {
              acc_tan = prevAcc[i];
            }
          }
          std::cout << "msg acc_tan: " << acc_tan << std::endl;
          
            
            
          int lane = round((pos_d - 2.0) / 4.0);
          
          double lane1AheadDist = 999;
          double lane2AheadDist = 999;
          double lane3AheadDist = 999;
          double lane1BehindDist = 999;
          double lane2BehindDist = 999;
          double lane3BehindDist = 999;
          double lane1AheadSpd = speed;
          double lane2AheadSpd = speed;
          double lane3AheadSpd = speed;
          double lane1BehindSpd = speed;
          double lane2BehindSpd = speed;
          double lane3BehindSpd = speed;

          double targetX_speed;
          double sf_s;
          double future_s;
          double sf_d;
          double sf_vx;
          double sf_vy;
          
          for (int i=0; i<sensor_fusion.size(); ++i) {
            //std::cout<<sensor_fusion[i][0]<<std::endl;
            sf_d=sensor_fusion[i][6];
            sf_s = sensor_fusion[i][5];
            sf_vx = sensor_fusion[i][3];
            sf_vy = sensor_fusion[i][4];
            targetX_speed = sqrt(pow(sf_vx, 2) + pow(sf_vy, 2));
            future_s = sf_s + ts * prevPathSize*targetX_speed;
            while (abs(future_s - pos_s) > max_dist) {
              std::cout << "too big dist" << std::endl;
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
                }
              }
              else {
                if ((pos_s - future_s) < lane1BehindDist) {
                  lane1BehindDist = pos_s - future_s;
                  lane1BehindSpd = targetX_speed;
                }
              }
            }
            else if (sf_d < 8) { //lane 2
              if (future_s > pos_s) {
                if ((future_s - pos_s) < lane2AheadDist) {
                  lane2AheadDist = future_s - pos_s;
                  lane2AheadSpd = targetX_speed;
                }
              }
              else {
                if ((pos_s - future_s) < lane2BehindDist) {
                  lane2BehindDist = pos_s - future_s;
                  lane2BehindSpd = targetX_speed;
                }
              }
            }
            else if (sf_d>=8) { //lane 3
              if (future_s > pos_s) {
                if ((future_s - pos_s) < lane3AheadDist) {
                  lane3AheadDist = future_s - pos_s;
                  lane3AheadSpd = targetX_speed;
                }
              }
              else {
              if ((pos_s - future_s) < lane3BehindDist) {
                lane3BehindDist = pos_s - future_s;
                lane3BehindSpd = targetX_speed;
              }
              }
            }
          }
          double car_ahead_dist = 999;
          double car_ahead_speed = 999;
          bool changeRightFeas = false;
          bool changeLeftFeas = false;
          if (lane == 0) {
            car_ahead_dist = lane1AheadDist;
            car_ahead_speed = lane1AheadSpd;
            if (((lane2BehindDist/(lane2BehindSpd-speed) > 3.0)||((lane2BehindDist / (lane2BehindSpd - speed) < 0))) && (lane2BehindDist > 15) && (lane2AheadSpd > lane1AheadSpd + 2) && (lane2AheadDist > 75) && (speed > 10)) {
              changeRightFeas = true;
            }
          }
          else if (lane == 1) {
            car_ahead_dist = lane2AheadDist;
            car_ahead_speed = lane2AheadSpd;
            if (((lane3BehindDist / (lane3BehindSpd - speed) > 3.0) || ((lane3BehindDist / (lane3BehindSpd - speed) < 0))) && (lane3BehindDist>15) && (lane3AheadDist > car_ahead_dist) && (speed > 10)) {
              changeRightFeas = true;
            }
            else if (((lane1BehindDist / (lane1BehindSpd - speed) > 3.0) || ((lane1BehindDist / (lane1BehindSpd - speed) < 0))) && (lane1BehindDist > 15) && (lane1AheadDist > car_ahead_dist) && (speed > 10)) {
              changeLeftFeas = true;
            }
          }
          else if (lane > 1) {
            car_ahead_dist = lane3AheadDist;
            car_ahead_speed = lane3AheadSpd;
            if (((lane2BehindDist / (lane2BehindSpd - speed) > 3.0) || ((lane2BehindDist / (lane2BehindSpd - speed) < 0))) && (lane2BehindDist > 15) && (lane2AheadDist > car_ahead_dist) && (speed > 10)) {
              changeLeftFeas = true;
            }
          }
          if (abs(acc_tan) > 1.0) {
            changeLeftFeas = false;
            changeRightFeas = false;
          }
          if ((changeLeftFeas) && (!commitChangeR)) {
            if (lane > 1) {
              --lane;
              commitChangeL = true;
            }

          }
          else if ((changeRightFeas) && (!commitChangeL)) {
            if (lane < 2) {
              ++lane;
              commitChangeR = true;
            }

          }
          else if (commitChangeL) {
            if (lane > 1) {
              --lane;
            }
          }
          else if (commitChangeR) {
            if (lane < 2) {
              ++lane;
            }
          }


          if (car_ahead_speed < ref_speed) {
            double ttc = (ref_speed - car_ahead_speed) / car_ahead_dist;
            if (ttc < 3.0) {
              ref_speed = car_ahead_speed;
            }
            if (ttc < 2.0) {
              ref_speed = car_ahead_speed - 2.5*maxDistTravel;
            }
            //std::cout << "car ahead slowdown: speed = " << car_ahead_speed << std::endl;
            //ref_speed = car_ahead_speed - 2.5*maxDistTravel;

          }
          else {
            ref_speed = 45.0 * maxDistTravel;
            //std::cout << "car ahead no slowdown: speed = " << car_ahead_speed << std::endl;

          }
          std::cout << "ref_speed " << ref_speed << std::endl;
          max_speed = ref_speed + 2.0*maxDistTravel;
          min_speed = ref_speed - 2.0*maxDistTravel;
          
          int projSteps = 5;
          int targSteps = 100;
          vector<double> xPath(projSteps), yPath(projSteps);
          //double tempS=pos_s;
          //double tempD= pos_d;
          //vector<double> xyTemp= getXY(tempS, tempD,map_waypoints_s, map_waypoints_x, map_waypoints_y);
          double xOffset = pos_x;
          double yOffset = pos_y;
          xPath[0] = 0;
          yPath[0] = 0;
          double shift = lane * 4.0 + 2.0 - pos_d;
          
          
          for (int i = 1; i < projSteps; ++i) {
            double stage = 1.0 / (projSteps - i);
            double tempS = pos_s + i * (speed + 5)*targSteps*stage*ts;
            
            double tempD = pos_d + shift;
            if ((commitChangeL) || (commitChangeR)) {
              
              if (abs(shift) > 3.0) {
                tempD = pos_d + shift / abs(shift)*.15;
              }
              else if (abs(shift) > 0.5) {
                tempD = pos_d + shift / abs(shift)*.4;
              }
              else {
                tempD = pos_d + shift / abs(shift)*.15; 
                commitChangeL = false;
                commitChangeR = false;
              }

              
            }
            //double tempD= pos_d+multiplier/(projSteps-1.0)*shift*stage*2;
            vector<double> xyTemp= getXY(tempS, tempD,map_waypoints_s, map_waypoints_x, map_waypoints_y);
            xPath[i]=xyTemp[0]-xOffset;
            yPath[i]=xyTemp[1]-yOffset;
            
          }
          double thetaRotCW=atan2(yPath[projSteps-1],xPath[projSteps-1]);
          vector<double> xTransPath(projSteps), yTransPath(projSteps);
          xTransPath[0] = 0;
          yTransPath[0] = 0;
          for (int i=1; i < projSteps; ++i) {
            vector<double> cwRot = rotateCW(xPath[i], yPath[i], thetaRotCW);
            xTransPath[i] = cwRot[0];
            yTransPath[i]= cwRot[1];
            if (xTransPath[i]<=xTransPath[i-1]) {
              //std::cout << "non-sorted" << std::endl;
            }
              
          }
          
          double pos_x_trans=0;
          double pos_y_trans=0;
          tk::spline s;
          s.set_points(xTransPath,yTransPath);
          std::cout<<"incoming speed: " << speed <<std::endl;
          //int record = (50 - oldMsgLen)*2;
          prevX.clear();
          prevY.clear();
          prevAcc.clear();
          for (int i =0; i < 50 - prevPathSize; ++i) {
            std::cout << "tangential acceleration incoming: " << acc_tan <<std::endl;
            bool underspeed = (speed <= min_speed);
            bool overspeed = (speed >= max_speed);
            bool overAcc = (acc_tan > 3.0);
            bool coastDown = (acc_tan >= sqrt(14.0*(abs(speed-ref_speed))));
            bool coastUp = (-acc_tan >= sqrt(6.0 * abs(speed - ref_speed)));
            if (underspeed) {
              if ((!overAcc) && (!coastDown)) {
                acc_tan = std::min(3.0, acc_tan + ts*7.0);
                std::cout << "jerk up" << std::endl;
              }
              
              if (overAcc) {
                acc_tan = 3.0;
                //std::cout << "overaccelerating, capped" << std::endl;
              }

              if (coastDown) {
                acc_tan =std::max(-1.0, acc_tan-7.0 * ts);
                //std::cout << "jerk down" << std::endl;
              }
            } else if (overspeed ) {
              if (overAcc) {
                acc_tan = 3.0;
              }
              
              if (coastUp) {
                acc_tan = std::min(1.0, acc_tan + 3.0*ts);
                //std::cout<<"coast up"<< std::endl;
              }
              else {
                acc_tan = std::max(-7.0, acc_tan - 8.0*ts);
                //std::cout << "brake" << std::endl;
              }
            } else {
              if (coastDown) {
                acc_tan = std::max(-1.0, acc_tan - 7.0 * ts);
                //std::cout << "coast down" << std::endl;
              } else if (coastUp) {
                  acc_tan = std::min(1.0, acc_tan + 3.0*ts);
                  //std::cout << "coast up" << std::endl;
              } else if (abs(acc_tan) < 1.5 * ts) {
                acc_tan = 0;
                //std::cout<<"dandy" << std::endl;
              }
            }


          
            acc_tan = std::max(acc_tan, -7.0);
            acc_tan = std::min(acc_tan, 3.0);
            if (speed < 0) {
              //std::cout<<"negative speed"<<std::endl;
            }
            std::cout<< "tangential acceleration out: " <<acc_tan <<std::endl;
            speed += acc_tan * ts;
            //std::cout << "speed out: " << speed << std::endl;
            double allowableDiff=0.03;
            double tempX=pos_x_trans+speed*ts;
            double tempY=s(tempX);
            double tempSpeed=speedCalc(tempX, tempY, pos_x_trans,pos_y_trans,ts);
            bool lastOverspeed = true;
            double incrementer = 0.01;
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
            double overageFactor = speed / tempSpeed;
            double deltaX = (tempX-pos_x_trans)*overageFactor;
            double deltaY = (tempY-pos_y_trans)*overageFactor;
            prev_pos_x=pos_x;
            prev_pos_y=pos_y;
            double arg1 = deltaX + pos_x_trans;
            double arg2 = deltaY + pos_y_trans;
            vector<double> rotCCW = rotateCCW(arg1, arg2, thetaRotCW);
            pos_x = rotCCW[0]+xOffset;
            pos_y= rotCCW[1]+yOffset;
            pos_x_trans+=deltaX;
            pos_y_trans+=deltaY;
            std::cout << "x: " << pos_x << " y: " << pos_y << std::endl;
            next_x_vals.push_back(pos_x);
            next_y_vals.push_back(pos_y);
            if (next_x_vals.size() < 3) {
              vector<double> kine2p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, ts);
              speed = kine2p[0];

              theta = kine2p[1];
              
              lastSpeed = speed-acc_tan*ts;
              //std::cout << "speed post-calculated: " << speed << std::endl;
            } else {
              prev_prev_pos_x=next_x_vals[next_x_vals.size()-3];
              prev_prev_pos_y=next_y_vals[next_y_vals.size()-3];
              vector<double> kine3p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, prev_prev_pos_x, prev_prev_pos_y, ts);
              
              speed = kine3p[0];
              theta = kine3p[1];
              lastSpeed = kine3p[2];
              acc_tan = kine3p[3];
              //std::cout << "speed post-calculated: " << speed << std::endl;
            }
            vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
            pos_s = frenetPos[0];
            pos_d = frenetPos[1];
            prevX.push_back(pos_x);
            prevY.push_back(pos_y);
            prevAcc.push_back(acc_tan);
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
#ifdef UWS_VCPKG
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }  // end websocket if
  }); // end h.onMessage


#ifdef UWS_VCPKG
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
#endif

#ifdef UWS_VCPKG
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
#ifdef UWS_VCPKG
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
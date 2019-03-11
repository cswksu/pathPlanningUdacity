#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
   double startX=start[0];
   double startXdot=start[1];
   double startXdot_dot=start[2];
   double endX=end[0];
   double endXdot=end[1];
   double endXdot_dot=end[2];
   
   double T2=pow(T,2);
   double T3=pow(T,3);
   double T4=pow(T,4);
   double T5=pow(T,5);
   
   MatrixXd A(3,3);
   A << T3, T4, T5,
   3*T2, 4*T3, 5*T4,
   6*T, 12*T2, 20*T3;
   //std::cout<<A<<std::endl;
   double b1=endX-(startX+startXdot*T+0.5*startXdot_dot*T2);
   double b2=endXdot - (startXdot+startXdot_dot*T);
   double b3=endXdot_dot-startXdot_dot;
   
   VectorXd b(3);
   b << b1, b2, b3;
   //std::cout<<b<<std::endl;
   VectorXd x(3);
   x =A.colPivHouseholderQr().solve(b);
   //std::cout<<x<<std::endl;

  return {startX,startXdot,startXdot_dot/2,x[0],x[1],x[2]};
}

int main() {
  uWS::Hub h;

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
  

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  if (in_map_.good()) {
    //std::cout << "file exists" << std::endl;
  }
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
    &map_waypoints_dx, &map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
      uWS::OpCode opCode) {
#else
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
    &map_waypoints_dx, &map_waypoints_dy]
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
          std::cout<<"msg received"<<std::endl;
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"]*0.44704;
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

          prevPathSize=std::min(prevPathSize,15);
          std::cout << "Previous path size: " << prevPathSize << std::endl;
          for (int i =0; i < prevPathSize; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          if (prevPathSize==0) {
            pos_x = car_x;
            pos_y = car_y;
            theta=deg2rad(car_yaw);
            pos_s=car_s;
            pos_d=car_d;
            speed = car_speed;
            pos_s = car_s;
            pos_d = car_d;
          } else if (prevPathSize == 1) {
            speed = car_speed;
            theta = deg2rad(car_yaw);
            pos_x = previous_path_x[0];
            pos_y = previous_path_y[0];
            pos_s = car_s;
            pos_d = car_d;
          } else if (prevPathSize == 2) {
            pos_x= previous_path_x[prevPathSize - 1];
            pos_y = previous_path_y[prevPathSize - 1];
            
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

            prev_pos_x = previous_path_x[prevPathSize - 2];
            prev_pos_y = previous_path_y[prevPathSize - 2];
            prev_prev_pos_x = previous_path_x[prevPathSize - 3];
            prev_prev_pos_y = previous_path_y[prevPathSize - 3];
            vector<double> kine3p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, prev_prev_pos_x, prev_prev_pos_y, ts);
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
            
            
          int lane = (pos_d - 2.0) / 4;
          
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
            else { //lane 3
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
          double car_ahead_dist;
          double car_ahead_speed;
          if (lane == 1) {
            car_ahead_dist = lane1AheadDist;
            car_ahead_speed = lane1AheadSpd;
          }
          else if (lane == 2) {
            car_ahead_dist = lane2AheadDist;
            car_ahead_speed = lane2AheadSpd;
          }
          else {
            car_ahead_dist = lane3AheadDist;
            car_ahead_speed = lane3AheadSpd;
          }
          if ((car_ahead_dist < 50) && (car_ahead_speed < ref_speed)) {
            std::cout << "car ahead slowdown: speed = " << car_ahead_speed << std::endl;
            ref_speed = car_ahead_speed - 2.0*maxDistTravel;
          }
          else {
            ref_speed = 45.0 * maxDistTravel;
          }

          max_speed = ref_speed + 2.0*maxDistTravel;
          min_speed = ref_speed - 2.0*maxDistTravel;
          int numSteps=30;
          int projSteps = 5;
          vector<double> xPath(projSteps), yPath(projSteps);
          double tempS=pos_s;
          double tempD= pos_d;
          vector<double> xyTemp= getXY(tempS, tempD,map_waypoints_s, map_waypoints_x, map_waypoints_y);
          double xOffset=xyTemp[0];
          double yOffset=xyTemp[1];
          xPath[0] = 0;
          yPath[0] = 0;
          for (int i=1; i < projSteps; ++i) {
            double tempS=pos_s+i*(speed+5)*numSteps*ts;
            double tempD= 6.0;
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
              std::cout << "non-sorted" << std::endl;
            }
              
          }
          
          double pos_x_trans=0;
          double pos_y_trans=0;
          tk::spline s;
          s.set_points(xTransPath,yTransPath);
          std::cout<<"incoming speed: " << speed <<std::endl;
          for (int i =0; i < 50 - prevPathSize; ++i) {
            std::cout << "tangential acceleration incoming: " << acc_tan <<std::endl;
            bool underspeed = (speed < min_speed);
            bool overspeed = (speed > max_speed);
            bool overAcc = (acc_tan > 5.0);
            bool coastDown = (acc_tan >= sqrt(14.0*(speed-ref_speed)));
            if (underspeed) {
              if ((!overAcc) && (!coastDown)) {
                acc_tan = std::min(5.0, acc_tan + ts*5.0);
                std::cout << "jerk up" << std::endl;
              } else if (overAcc) {
                acc_tan = 5.0;
                std::cout << "overaccelerating, capped" << std::endl;
              } else if (coastDown) {
                acc_tan =std::max(-3.0, acc_tan-7.0 * ts);
                std::cout << "jerk down" << std::endl;
              }
            } else if (overspeed ) {
              if (overAcc) {
                acc_tan = 5.0;
              }
              acc_tan =std::max(-7.0,acc_tan-8.0*ts);
              std::cout << "brake" << std::endl;
            } else {
              if (coastDown) {
                acc_tan = std::max(-3.0, acc_tan - 7.0 * ts);
                std::cout << "jerk down" << std::endl;
              }
            }
            if (speed < 0) {
              std::cout<<"negative speed"<<std::endl;
            }
            std::cout<< "tangential acceleration out: " <<acc_tan <<std::endl;
            speed += acc_tan * ts;
            std::cout << "speed out: " << speed << std::endl;
            double allowableDiff=0.02;
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
            next_x_vals.push_back(pos_x);
            next_y_vals.push_back(pos_y);
            if (next_x_vals.size() < 3) {
              vector<double> kine2p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, ts);
              speed = kine2p[0];

              theta = kine2p[1];
              vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
              pos_s = frenetPos[0];
              pos_d = frenetPos[1];
              lastSpeed = speed-acc_tan*ts;
              std::cout << "speed post-calculated: " << speed << std::endl;
            } else {
              prev_prev_pos_x=next_x_vals[next_x_vals.size()-3];
              prev_prev_pos_y=next_y_vals[next_y_vals.size()-3];
              vector<double> kine3p = kinematics(pos_x, pos_y, prev_pos_x, prev_pos_y, prev_prev_pos_x, prev_prev_pos_y, ts);
              
              speed = kine3p[0];
              theta = kine3p[1];
              lastSpeed = kine3p[2];
              acc_tan = kine3p[3];
              std::cout << "speed post-calculated: " << speed << std::endl;
            }
            vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
            pos_s = frenetPos[0];
            pos_d = frenetPos[1];
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
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


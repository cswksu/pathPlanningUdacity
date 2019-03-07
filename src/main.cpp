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
          double maxDistTravel=0.42; // distance in meters to travel
          double ref_speed = 50.0 * maxDistTravel;
          double max_speed = ref_speed+0.15;
          double min_speed = ref_speed-0.15;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

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

          
          int lane = (car_d-2.0)/4;
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
          double prev3_pos_x;
          double prev3_pos_y;
          double theta;
          double acc_x;
          double acc_y;
          double v_x;
          double v_y;
          double speed;
          double acc;
          
          double rCurve;
          double jerk_x=0;
          double jerk_y=0;
          double jerk=0;
          double lastSpeed = 0;

          prevPathSize=std::min(prevPathSize,15);
          for (int i =0; i < prevPathSize; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          if (prevPathSize==0) {
            pos_x = car_x;
            std::cout<<"car's initial x position: " << pos_x<<std::endl;
            pos_y = car_y;
            std::cout<<"car's initial y position: " << pos_y<<std::endl;
            theta=deg2rad(car_yaw);
            pos_s=car_s;
            pos_d=car_d;
            acc_x=0;
            acc_y=0;
            v_x=car_speed*cos(theta);
            v_y=car_speed*sin(theta);
            acc = 0;
            rCurve = 9999;
            speed = car_speed;
          } else {
            acc=0;
            speed=0;
            pos_x = previous_path_x[prevPathSize-1];
            pos_y = previous_path_y[prevPathSize-1];
            theta=deg2rad(car_yaw);
            if (prevPathSize>1) {
              
              prev_pos_x = previous_path_x[prevPathSize-2];
              prev_pos_y = previous_path_y[prevPathSize-2];
              v_x=(pos_x-prev_pos_x)/0.02;
              v_y=(pos_y-prev_pos_y)/0.02;
              theta = atan2(pos_y-prev_pos_y,pos_x-prev_pos_x);
              speed = sqrt(v_x*v_x + v_y * v_y);
              lastSpeed = speed;
              rCurve = 9999;
              if (prevPathSize>2) {
                prev_prev_pos_x=previous_path_x[prevPathSize-3];
                prev_prev_pos_y=previous_path_y[prevPathSize-3];
                acc_x=(pos_x-2*prev_pos_x+prev_prev_pos_x)/(0.02*0.02);
                acc_y=(pos_y-2*prev_pos_y+prev_prev_pos_y)/(0.02*0.02);
                acc = sqrt(acc_x*acc_x+acc_y*acc_y);
                lastSpeed=sqrt(pow(prev_pos_x-prev_prev_pos_x,2)+pow(prev_pos_y-prev_prev_pos_y,2))/0.02;
                rCurve = abs(pow(speed, 3) / (v_x*acc_y - v_y * acc_x));
                if (prevPathSize > 3) {
                  prev3_pos_x= previous_path_x[prevPathSize - 4];
                  prev3_pos_y = previous_path_y[prevPathSize - 4];
                  jerk_x = -pos_x + 3 * prev_pos_x - 3 * prev_prev_pos_x + prev3_pos_x;
                  jerk_y = -pos_y + 3 * prev_pos_y - 3 * prev_prev_pos_y + prev3_pos_y;
                  jerk = sqrt(jerk_x*jerk_x + jerk_y * jerk_y);
                }
              }
              
            }
            
            vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
            pos_s=frenetPos[0];
            pos_d=frenetPos[1];
          }
          
          double car_ahead_speed=999;
          double car_ahead_dist=999;
          double sf_s;
          double sf_d;
          double sf_vx;
          double sf_vy;
          double tempSpeed;
          /*
          for (int i=0; i<sensor_fusion.size(); ++i) {
            //std::cout<<sensor_fusion[i][0]<<std::endl;
            sf_d=sensor_fusion[i][6];
            if (abs(sf_d-car_d)<4) {
              //same lane
              sf_s=sensor_fusion[i][5];
              if (sf_s-car_s>0) {
                if (sf_s-car_s<car_ahead_dist) {
                  car_ahead_dist=sf_s-car_s;
                  sf_vx=sensor_fusion[i][3];
                  sf_vy=sensor_fusion[i][4];
                  car_ahead_speed=sqrt(pow(sf_vx,2)+pow(sf_vy,2));
                }
              }
            }
          }

          if ((car_ahead_dist < 50) && (car_ahead_speed < ref_speed)) {
            ref_speed = car_ahead_speed - 0.5;
            max_speed = ref_speed + 0.15;
            min_speed = ref_speed - 0.15;
          }
          else {
            ref_speed = 50.0 * maxDistTravel;
            max_speed = ref_speed + 0.15;
            min_speed = ref_speed - 0.15;
          } */

          int numSteps=25;
          int projSteps=std::max(3,(50-prevPathSize)/numSteps);
          vector<double> xPath(projSteps), yPath(projSteps);
          double tempS=pos_s;
          double tempD= 6.0;
          vector<double> xyTemp= getXY(tempS, tempD,map_waypoints_s, map_waypoints_x, map_waypoints_y);
          double xOffset=xyTemp[0];
          double yOffset=xyTemp[1];
          for (int i=0; i < projSteps; ++i) {
            double tempS=pos_s+i*(speed+5)*numSteps*0.02;
            double tempD= 6.0;
            vector<double> xyTemp= getXY(tempS, tempD,map_waypoints_s, map_waypoints_x, map_waypoints_y);
            xPath[i]=xyTemp[0]-xOffset;
            yPath[i]=xyTemp[1]-yOffset;
            
          }
          double thetaRotCW=atan2(yPath[projSteps-1]-yPath[0],xPath[projSteps-1]-xPath[0]);
          vector<double> xTransPath(projSteps), yTransPath(projSteps);
          for (int i=0; i < projSteps; ++i) {
            xTransPath[i]=xPath[i]*cos(thetaRotCW)+yPath[i]*sin(thetaRotCW);
            yTransPath[i]=-xPath[i]*sin(thetaRotCW)+yPath[i]*cos(thetaRotCW);
            if (i > 0) {
              if (xTransPath[i]<=xTransPath[i-1]) {
                std::cout << "non-sorted" << std::endl;
              }
              
            }
          }
          double pos_x_trans=0;
          double pos_y_trans=0;
          tk::spline s;
          s.set_points(xTransPath,yTransPath);
          std::cout<<"incoming speed: " << speed <<std::endl;
          for (int i =0; i < 50 - prevPathSize; ++i) {
            double acc_cent = pow(speed, 2) / rCurve;
            double acc_tan = (speed - lastSpeed)/0.02;
            std::cout << "tangential acceleration incoming: " << acc_tan <<std::endl;
            bool underspeed = (speed < min_speed);
            bool overspeed = (speed > max_speed);
            bool overAcc = (abs(acc_tan) > 5.0);
            bool coastDown = (acc_tan >= sqrt(abs(14.0*(min_speed - speed))));
            if (underspeed) {
              if ((!overAcc) && (!coastDown)) {
                acc_tan = std::min(5.0, acc_tan + 0.02*5.0);
                std::cout << "jerk up" << std::endl;
              } else {//if ((acc_tan >= 5.0)||(acc_tan>=sqrt(abs(10.0*(min_speed-speed))))) {
                acc_tan =std::max(-7.0, acc_tan-7.0 * 0.02);
                std::cout << "jerk down" << std::endl;
              }
            } else if (overspeed ) {
              acc_tan =std::max(-7.0,acc_tan-8.0*0.02);
              std::cout << "brake" << std::endl;
              //speed = max_speed;
            }
            //speed = 22;
            if (speed < 0) {
              std::cout<<"negative speed"<<std::endl;
            }
            std::cout:: "tangential acceleration out: " <<acc_tan <<std::endl;
            speed += acc_tan * 0.02;
            //std::cout << "speed requested: " << speed <<std::endl;
            /*else if (speed > max_speed) {
              speed = max_speed;
              std::cout << "overspeed warning" << std::endl;
            }*/
            
            double allowableDiff=0.02;
            double tempX=pos_x_trans+speed*0.02;
            double tempY=s(tempX);
            double tempSpeed=sqrt(pow(tempX-pos_x_trans,2)+pow(tempY-pos_y_trans,2))/0.02;
            //std::cout << "finding trajectory" << std::endl;
            bool lastOverspeed = true;
            double incrementer = 0.01;
            while(abs(tempSpeed-speed)>allowableDiff) {
              /*std::cout << "Temp Speed: " << tempSpeed << std::endl;
              std::cout << "Targ Speed: " << speed << std::endl;
              std::cout << "Difference: " << abs(tempSpeed - speed) << std::endl;*/
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
              //std::cout << tempX << std::endl;
              tempY=s(tempX);
              tempSpeed=sqrt(pow(tempX-pos_x_trans,2)+pow(tempY-pos_y_trans,2))/0.02;
              


            }
            //std::cout << "temp speed used: " << tempSpeed <<std::endl;
            //std::cout << "trajectory found" << std::endl;
            /*
            double transHdg=atan2(s(pos_x_trans+1.0)-pos_y_trans,1.0);
            double deltaXRot=speed*0.02*cos(transHdg);
            pos_x_trans+=deltaXRot;
            pos_y_trans=s(pos_x_trans);*/
            double deltaX = tempX-pos_x_trans;
            double deltaY = tempY-pos_y_trans;
            pos_x_trans=tempX;
            pos_y_trans=tempY;
            double oldPos_x=pos_x;
            double oldPos_y=pos_y;
            pos_x+=deltaX*cos(-thetaRotCW)+deltaY*sin(-thetaRotCW);
            //std::cout << "car's new x position: " << pos_x << std::endl;
            pos_y+= -deltaX*sin(-thetaRotCW)+deltaY*cos(-thetaRotCW);
            //std::cout << "car's new y position: " << pos_y << std::endl;
            
            /**pos_s+=speed*0.02;
            pos_d = 6.0;
            vector<double> xyTemp= getXY(pos_s, pos_d,map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            pos_x=xyTemp[0];
            pos_y=xyTemp[1];
            double rotPos_x=pos_x*cos(thetaRotCW)+pos_y*sin(thetaRotCW);
            double rotPos_y=s(rotPos_x);
            pos_y= -rotPos_x*sin(-thetaRotCW)+rotPos_y*cos(-thetaRotCW);
            **/
            /*double speedCheck = sqrt(std::pow((pos_x-oldPos_x)/0.02,2)+std::pow((pos_y-oldPos_y)/0.02,2));
            if (speedCheck > max_speed) {
              
              double overageRatio=speedCheck/max_speed;
              std::cout << "overspeed: " << overageRatio << std::endl;
              pos_x=oldPos_x+(pos_x-oldPos_x)/overageRatio;
              pos_y=oldPos_y+(pos_y-oldPos_y)/overageRatio;
              pos_x_trans=pos_x*cos(thetaRotCW)+pos_y*sin(thetaRotCW);
              pos_y_trans=-pos_x*sin(thetaRotCW)+pos_y*cos(thetaRotCW);
            }*/
            next_x_vals.push_back(pos_x);
            next_y_vals.push_back(pos_y);
            prev_pos_x = oldPos_x;
            prev_pos_y = oldPos_y;
            theta = atan2(pos_y-prev_pos_y,pos_x-prev_pos_x);
            vector<double> frenetPos = getFrenet(pos_x, pos_y, theta, map_waypoints_x, map_waypoints_y);
            pos_s=frenetPos[0];
            pos_d=frenetPos[1];
            lastSpeed = sqrt(v_x*v_x+v_y*v_y);
            v_x=(pos_x-prev_pos_x)/0.02;
            v_y=(pos_y-prev_pos_y)/0.02;
            speed = sqrt(v_x*v_x + v_y * v_y);
            std::cout << "speed post-calculated: " << speed << std::endl;
            if (next_x_vals.size()>2) {
              prev_prev_pos_x=next_x_vals[next_x_vals.size()-3];
              prev_prev_pos_y=next_y_vals[next_y_vals.size()-3];
              acc_x=(pos_x-2*prev_pos_x+prev_prev_pos_x)/(0.02*0.02);
              acc_y=(pos_y-2*prev_pos_y+prev_prev_pos_y)/(0.02*0.02);
              rCurve = abs(pow(speed, 3) / (v_x*acc_y - v_y * acc_x));
              if (next_x_vals.size() > 3) {
                prev3_pos_x = next_x_vals[next_x_vals.size() - 4];
                prev3_pos_y = next_y_vals[next_x_vals.size() - 4];
                jerk_x = -pos_x + 3 * prev_pos_x - 3 * prev_prev_pos_x + prev3_pos_x;
                jerk_y = -pos_y + 3 * prev_pos_y - 3 * prev_prev_pos_y + prev3_pos_y;
                jerk = sqrt(jerk_x*jerk_x + jerk_y * jerk_y);
              }
            }
            
            acc = sqrt(acc_x*acc_x+acc_y*acc_y);
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


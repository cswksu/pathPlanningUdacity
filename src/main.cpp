#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

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

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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
          double car_speed = j[1]["speed"];

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

          double maxDistTravel=0.42; // distance in meters to travel
          int lane = (car_d-2.0)/4;
          //std::cout<<lane<<std::endl;
          int prevPathSize=previous_path_x.size();
          double pos_x;
          double pos_y;
          double prev_pos_x;
          double prev_pos_y;
          double prev_prev_pos_x;
          double prev_prev_pos_y;
          double theta;
          double acc_x;
          double acc_y;
          double v_x;
          double v_y;
          double speed;


          for (int i =0; i < prevPathSize; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          if (prevPathSize==0) {
            pos_x = car_x;
            pos_y = car_y;
            theta=deg2rad(car_yaw);
            acc_x=0;
            acc_y=0;
            v_x=0;
            v_y=0;
          } else {
            pos_x = previous_path_x[prevPathSize-1];
            pos_y = previous_path_y[prevPathSize-1];
            prev_pos_x = previous_path_x[prevPathSize-2];
            prev_pos_y = previous_path_y[prevPathSize-2];
            prev_prev_pos_x=previous_path_x[prevPathSize-3];
            prev_prev_pos_y=previous_path_y[prevPathSize-3];
            theta = atan2(pos_y-prev_pos_y,pos_x-prev_pos_x);
            v_x=(pos_x-prev_pos_x)/0.02;
            v_y=(pos_y-prev_pos_y)/0.02;
            acc_x=(pos_x-2*prev_pos_x+prev_prev_pos_x)/(0.02*0.02);
            acc_y=(pos_y-2*prev_pos_y+prev_prev_pos_y)/(0.02*0.02);
          }
          speed = sqrt(v_x*v_x+v_y*v_y);
          double car_ahead_speed=999;
          double car_ahead_dist=999;
          double sf_s;
          double sf_d;
          double sf_vx;
          double sf_vy;
          //double tempSpeed;
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
          double ref_speed;
          if (car_ahead_dist<50) {
            ref_speed=std::min(car_ahead_speed,maxDistTravel*50);
          } else {
            ref_speed=maxDistTravel*50;
          }
          double timeGoal;
          if (ref_speed-speed<0.5) {
            timeGoal=1; } else {
            timeGoal=(ref_speed-speed)/3;
          }
          double x_final;
          double x_dot_final;
          double y_final;
          double y_dot_final;
          double targ_s=car_s+(ref_speed+speed)/2*timeGoal;
          double targ_d=car_d;
          vector<double> targXY = getXY(targ_s, targ_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> targXYplusOne = getXY(targ_s+1,targ_d,map_waypoints_s, map_waypoints_x, map_waypoints_y);
          double dirTargX=targXYplusOne[0]-targXY[0];
          double dirTargY=targXYplusOne[1]-targXY[1];
          x_dot_final=dirTargX*ref_speed;
          y_dot_final=dirTargY*ref_speed;
          x_final=targXY[0];
          y_final=targXY[1];
          vector<double> startX={pos_x,v_x,acc_x};
          vector<double> startY={pos_y,v_y,acc_y};
          vector<double> endX={x_final,x_dot_final,0}
          vector<double> endY={y_final,y_dot_final,0}
          vector<double> trajX = JMT(startX, endX, timeGoal);
          vector<double> trajY = JMT(startY, endY, timeGoal);

          
          double t_iter=0;
          for (int i = 0; i < 50-prevPathSize; ++i) {
            t_iter=t_iter+0.02;
            next_x_vals.push_back(pos_x+v_x*t_iter+acc_x*pow(t_iter,2)+trajX[3]*pow(t_iter,3)+trajX[4]*pow(t_iter,4)+trajX[5]*pow(t_iter,5));
            next_y_vals.push_back(pos_y+v_y*t_iter+acc_y*pow(t_iter,2)+trajY[3]*pow(t_iter,3)+trajY[4]*pow(t_iter,4)+trajY[5]*pow(t_iter,5));
            //pos_x += v_x*t_iter+acc_x*pow(t_iter,2)+trajX[3]*pow(t_iter,3)+trajX[4]*pow(t_iter,4)+trajX[5]*pow(t_iter,5);
            //pos_y += v_y*t_iter+acc_y*pow(t_iter,2)+trajY[3]*pow(t_iter,3)+trajY[4]*pow(t_iter,4)+trajY[5]*pow(t_iter,5)
          }



          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}


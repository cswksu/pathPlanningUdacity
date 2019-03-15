#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <numeric>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
  const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
  const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
  const vector<double> &maps_x,
  const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y * n_y) / (n_x*n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return { frenet_s,frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
  const vector<double> &maps_x,
  const vector<double> &maps_y) {
  int prev_wp = -1;
  
  //switched order of arguments to utilize short-circuiting
  while ((prev_wp < (int)(maps_s.size() - 1)) && s > maps_s[prev_wp + 1]) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
    (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return { x,y };
}

//rotate point clockwise by given angle in radians (positive is CW)
vector<double> rotateCW(double& x, double& y, double& theta) {
  double xprime = x * cos(theta) + y * sin(theta);
  double yprime = -x * sin(theta) + y * cos(theta);

  return { xprime, yprime };
}

//rotate point counterclockwise by given angle in radians (positive is CCW)
vector<double> rotateCCW(double& x, double& y, double& theta) {
  double xprime = x * cos(theta) - y * sin(theta);
  double yprime = x * sin(theta) + y * cos(theta);

  return {xprime, yprime};
}

//calcluate speed from 2 points, timestep
double speedCalc(double x1, double& y1, double& x0, double& y0, double& ts) {
  return sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) / ts;
}

// Return speed, theta given (x1, y1), (x0, y0), and timestep.
vector<double> kinematics(double& x1, double& y1, double& x0, double& y0, double& ts) {
  double speed = speedCalc(x1, y1, x0, y0,ts);
  double theta = atan2(y1 - y0, x1 - x0);
  return { speed, theta };
}

// Return speed, theta, last speed, accT for 3 points, timestep
vector<double> kinematics(double& x2, double& y2, double& x1, double& y1, double& x0, double& y0, double& ts) {
  double speed = speedCalc(x2, y2, x1, y1,ts);
  double theta = atan2(y2 - y1, x2 - x1);

  double speed0 = speedCalc(x1, y1, x0, y0, ts);
  double accT = (speed - speed0) / ts;

  return { speed, theta, speed0, accT };
}


//calculate speed, theta, last speed, accT for vectors of points, timestep
vector<double> kinematics(vector<double> xPath, vector<double> yPath, double& ts) {
  int len = xPath.size(); //length of paths
  double theta = atan2(yPath[len - 1] - yPath[len - 2], xPath[len - 1] - xPath[len - 2]);
  vector<double> speedVect(len - 1); //lose 1 point for speed calculations

  for (int i = 0; i < len - 1; ++i) {
    speedVect[i] = sqrt(pow(xPath[i + 1] - xPath[i], 2) + pow(yPath[i + 1] - yPath[i], 2)) / ts;
    //calculate speed for each pair of points
  }

  double acc_tan = 0; //tangential acceleration
  switch (len - 2) { //switch on number of points in path
  //calculate tangential acceleration using backwards finite differences of speed
    case 2:
      acc_tan = (speedVect[1] - speedVect[0]) / ts;
      break;
    case 3:
      acc_tan = (1.5*speedVect[2] - 2.0*speedVect[1] + 0.5*speedVect[0]) / ts;
      break;
    case 4:
      acc_tan = (11.0 / 6.0*speedVect[3] - 3.0*speedVect[2] + 1.5*speedVect[1] - speedVect[0] / 3.0) / ts;
      break;
    case 5:
      acc_tan = (25.0/12.0*speedVect[4]-4.0*speedVect[3]+3.0*speedVect[2]-4.0/3.0*speedVect[1]+0.25*speedVect[0]) / ts;
      break;
    case 6:
      acc_tan= (137.0/60.0*speedVect[5]-5.0*speedVect[4]+5.0*speedVect[3]-10.0/3.0*speedVect[2]+1.25*speedVect[1]-0.2*speedVect[0])/ts;
      break;
    default:
      acc_tan=(49.0/20.0*speedVect[len-2]-6.0*speedVect[len-3]+7.5*speedVect[len-4]-20.0/3.0*speedVect[len-5]+3.75*speedVect[len-6]-1.2*speedVect[len-7]+speedVect[len-8]/6.0)/ts;
      break;
  }
    
  return { speedVect[len - 2],theta,speedVect[len - 3],acc_tan };
}

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
  double startX = start[0];
  double startXdot = start[1];
  double startXdot_dot = start[2];
  double endX = end[0];
  double endXdot = end[1];
  double endXdot_dot = end[2];

  double T2 = pow(T, 2);
  double T3 = pow(T, 3);
  double T4 = pow(T, 4);
  double T5 = pow(T, 5);

  MatrixXd A(3, 3);
  A << T3, T4, T5,
    3 * T2, 4 * T3, 5 * T4,
    6 * T, 12 * T2, 20 * T3;
  //std::cout<<A<<std::endl;
  double b1 = endX - (startX + startXdot * T + 0.5*startXdot_dot*T2);
  double b2 = endXdot - (startXdot + startXdot_dot * T);
  double b3 = endXdot_dot - startXdot_dot;

  VectorXd b(3);
  b << b1, b2, b3;
  //std::cout<<b<<std::endl;
  VectorXd x(3);
  x = A.colPivHouseholderQr().solve(b);
  //std::cout<<x<<std::endl;

  return { startX,startXdot,startXdot_dot / 2,x[0],x[1],x[2] };
}


#endif  // HELPERS_H

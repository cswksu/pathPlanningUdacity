#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

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

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
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

vector<double> rotateCW(double& x, double& y, double& theta) {
  double xprime = x * cos(theta) + y * sin(theta);
  double yprime = -x * sin(theta) + y * cos(theta);

  return { xprime, yprime };
}

vector<double> rotateCCW(double& x, double& y, double& theta) {
  double xprime = x * cos(theta) - y * sin(theta);
  double yprime = x * sin(theta) + y * cos(theta);

  return {xprime, yprime};
}

double speedCalc(double x1, double& y1, double& x0, double& y0, double& ts) {
  return sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) / ts;
}

// Return speed, theta given (x1, y1), (x0, y0), and timestep.
vector<double> kinematics(double& x1, double& y1, double& x0, double& y0, double& ts) {
  double speed = speedCalc(x1, y1, x0, y0,ts);
  double theta = atan2(y1 - y0, x1 - x0);
  return { speed, theta };
}

// Return speed, theta, last speed, accT
vector<double> kinematics(double& x2, double& y2, double& x1, double& y1, double& x0, double& y0, double& ts) {
  double vx=0.5*(3.0*x2-4.0*x1+x0)/ts;
  double vy=0.5*(3.0*y2-4.0*y1+y0)/ts;
  double theta = atan2(vy, vx);
  double speed = sqrt(vx*vx+vy*vy);
  vx=0.5*(x2-x0)/ts;
  vy=0.5*(y2-y0)/ts;
  double speed0 = sqrt(vx*vx+vy*vy);
  //double acc = sqrt(pow(vx - vx0, 2) + pow(vy - vy0, 2))/ts;
  double accT = (speed - speed0) / ts;
  //double accN = sqrt(pow(acc, 2) - pow(accT, 2));

  return { speed, theta, speed0, accT };
}

vector<double> kinematics(double& x3, double& y3, double& x2, double& y2, double& x1, double& y1, double& x0, double& y0, double& ts) {
  double vx=(1.0/6.0)*(11.0*x3-18.0*x2+9.0*x1-2.0*x0)/ts;
  double vy=(1.0/6.0)*(11.0*y3-18.0*y2+9.0*y1-2.0*y0)/ts;
  double theta = atan2(vy, vx);
  double speed2 = sqrt(vx*vx+vy*vy);
  vx=(1.0/6.0)*(2.0*x3+3.0*x2-6.0*x1+x0)/ts;
  vy=(1.0/6.0)*(2.0*y3+3.0*y2-6.0*y1+y0)/ts;
  double speed1 = sqrt(vx*vx+vy*vy);
  double speed0 = speedCalc(x1,y1,x0,y0,ts);
  //double acc = sqrt(pow(vx - vx0, 2) + pow(vy - vy0, 2))/ts;
  double accT = (1.5*speed2-2.0*speed1+0.5*speed0)/ts;

  return { speed2, theta, speed1, accT };
}

vector<double> kinematics(double& x4, double& y4, double& x3, double& y3, double& x2, double& y2, double& x1, double& y1, double& x0, double& y0, double& ts) {
  double vx=(1.0/12.0)*(25.0*x4-48.0*x3+36.0*x2-16.0*x1+3.0*x0)/ts;
  double vy=(1.0/12.0)*(25.0*y4-48.0*y3+36.0*y2-16.0*y1+3.0*y0)/ts;
  double theta = atan2(vy, vx);
  double speed3 = sqrt(vx*vx+vy*vy);
  vx=(1.0/12.0)*(3.0*x4+10.0*x3-18.0*x2+6.0*x1-1.0*x0)/ts;
  vy=(1.0/12.0)*(3.0*y4+10.0*y3-18.0*y2+6.0*y1-1.0*y0)/ts;
  double speed2 = sqrt(vx*vx+vy*vy);
  double speed1 = speedCalc(x2, y2, x1, y1, ts);
  double speed0 = speedCalc(x1,y1,x0,y0,ts);
  //double acc = sqrt(pow(vx - vx0, 2) + pow(vy - vy0, 2))/ts;
  double accT = (11.0/6.0*speed3-3.0*speed2+1.5*speed1-1.0/3.0*speed0)/ts;

  return { speed3, theta, speed2, accT };
}



vector<double> kinematics(double& x5, double& y5, double& x4, double& y4, double& x3, double& y3, double& x2, double& y2, double& x1, double& y1, double& x0, double& y0, double& ts) {
  double vx=(1.0/60.0)*(137.0*x5-300.0*x4+300.0*x3-200.0*x2+75.0*x1-12.0*x0)/ts;
  double vy=(1.0/60.0)*(137.0*y5-300.0*y4+300.0*y3-200.0*y2+75.0*y1-12.0*y0)/ts;
  double theta = atan2(vy, vx);
  double speed4 = sqrt(vx*vx+vy*vy);
  vx=(1.0/60.0)*(12.0*x5+65.0*x4-120.0*x3+60.0*x2-20.0*x1+3.0*x0)/ts;
  vy=(1.0/60.0)*(12.0*y5+65.0*y4-120.0*y3+60.0*y2-20.0*y1+3.0*y0)/ts;
  double speed3 = sqrt(vx*vx+vy*vy);
  double speed2 = speedCalc(x3, y3, x2, y2,ts);
  double speed1 = speedCalc(x2, y2, x1, y1, ts);
  double speed0 = speedCalc(x1,y1,x0,y0,ts);
  //double acc = sqrt(pow(vx - vx0, 2) + pow(vy - vy0, 2))/ts;
  double accT = (25.0/12.0*speed4-4.0*speed3+3.0*speed2-4.0/3.0*speed1+0.25*speed0)/ts;

  return { speed4, theta, speed3, accT };
}

vector<double> kinematics(double& x6, double& y6, double& x5, double& y5, double& x4, double& y4, double& x3, double& y3, double& x2, double& y2, double& x1, double& y1, double& x0, double& y0, double& ts) {
  double vx=(1.0/60.0)*(147.0*x6-360.0*x5+450.0*x4-400.0*x3+225.0*x2-72.0*x1+10.0*x0)/ts;
  double vy=(1.0/60.0)*(147.0*y6-360.0*y5+450.0*y4-400.0*y3+225.0*y2-72.0*y1+10.0*y0)/ts;
  double theta = atan2(vy, vx);
  double speed5 = sqrt(vx*vx+vy*vy);
  vx=(1.0/60.0)*(10.0*x6+77.0*x5-150.0*x4+100.0*x3-50.0*x2+15.0*x1-2.0*x0)/ts;
  vy=(1.0/60.0)*(10.0*y6+77.0*y5-150.0*y4+100.0*y3-50.0*y2+15.0*y1-2.0*y0)/ts;
  double speed4 = sqrt(vx*vx+vy*vy);
  double speed3 = speedCalc(x4,y4,x3,y3,ts);
  double speed2 = speedCalc(x3, y3, x2, y2,ts);
  double speed1 = speedCalc(x2, y2, x1, y1, ts);
  double speed0 = speedCalc(x1,y1,x0,y0,ts);
  //double acc = sqrt(pow(vx - vx0, 2) + pow(vy - vy0, 2))/ts;
  double accT = (137.0/60.0*speed5-5.0*speed4+5.0*speed3-10.0/3.0*speed2+1.25*speed1-0.2*speed0)/ts;

  return { speed5, theta, speed4, accT };
}


vector<double> kinematics(double& x7, double& y7, double& x6, double& y6, double& x5, double& y5, double& x4, double& y4, double& x3, double& y3, double& x2, double& y2, double& x1, double& y1, double& x0, double& y0, double& ts) {
  double vx = (1/420.0)*(1089.0*x7-2940.0*x6+4410.0*x5-4900.0*x4+3675.0*x3-1764.0*x2+490.0*x1-60.0*x0)/ts;
  double vy = (1/420.0)*(1089.0*y7-2940.0*y6+4410.0*y5-4900.0*y4+3675.0*y3-1764.0*y2+490.0*y1-60.0*y0)/ts;
  double theta = atan2(vy,vx);
  double speed6 = sqrt(vx*vx+vy*vy);
  vx = (1/420.0)*(60.0*x7+609.0*x6-1260.0*x5+1050.0*x4-700.0*x3+315.0*x2-84.0*x1+10.0*x0)/ts;
  vy = (1/420.0)*(60.0*y7+609.0*y6-1260.0*y5+1050.0*y4-700.0*y3+315.0*y2-84.0*y1+10.0*y0)/ts;
  double speed5 = sqrt(vx*vx+vy*vy);
  double speed4 = speedCalc(x5,y5,x4,y4,ts);
  double speed3 = speedCalc(x4,y4,x3,y3,ts);
  double speed2 = speedCalc(x3, y3, x2, y2,ts);
  double speed1 = speedCalc(x2, y2, x1, y1, ts);
  double speed0 = speedCalc(x1,y1,x0,y0,ts);
  //double acc = sqrt(pow(vx - vx0, 2) + pow(vy - vy0, 2))/ts;
  double accT = (49.0/20*speed6-6.0*speed5+7.5*speed4-20.0/3.0*speed3+3.75*speed2-1.2*speed1+1.0/6.0*speed0)/ts;

  return { speed6, theta, speed5, accT };
}


#endif  // HELPERS_H

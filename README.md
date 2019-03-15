# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Rubric Questions

### Compilation

*The code compiles correctly.*

Yes, the code compiles with CMake. O3 compilation is added to the makefile to increase speed.

### Valid Trajectories

*The car is able to drive at least 4.32 miles without incident*

The code runs for more than 4.32 miles successfully. See below run with 3 laps completed without issue: 

![proof](https://github.com/cswksu/pathPlanningUdacity/blob/master/images/proof.png)

*The car drives according to the speed limit.*

The car's nominal speed is 45 mph. If a simple parameter tweak could bump this higher if needed.

*Max Acceleration and Jerk are not Exceeded.*

*Car does not have collisions.*

*The car stays in its lane, except for the time between changing lanes.*

The car meets these criteria, as seen by the counter.

*The car is able to change lanes*

The car changes lanes. As seen in the proof, it ended up in the outside lane, and the car starts in the middle lane. Ergo, lane change.

### Reflection

#### Reception of old trajectory

The code takes in up to the last 15 points from the JSON message, and analyzes the trajectory for last known speed, tangential acceleration, and position. This is done with the overloaded kinematics functions I created in the helpers.cpp file. It was my experience that the process of sending a trajectory to the simulator and then receiving it back introduced a lot of noise, as the simulator converts all points to floats. As such, unacceptably high errors were introduced into my backwards finite difference calculations for acceleration, even when averaging or using up to 6th order finite difference functions. As such, the code retains its last trajectory as a double, and matches the last point in the trajectory to its corresponding tangential acceleration. 

#### Unpacking of sensor fusion

The code analzyes the sensor fusion package received from the perfect sensors and distills it into the following information:

* Lane 1 Ahead
* Lane 2 Ahead
* Lane 3 Ahead
* Lane 1 Behind
* Lane 2 Behind
* Lane 3 Behind

For each of these positions, distance, speed, and Time to Collision were calculated. The car ahead of the host/ego vehicle was also identified.

#### Lane change feasibility

Based on the host's lane, it checks other lanes for feasibility, checking the following:

* time since last lane change
* presence of curve ahead
* host tangential acceleration
* distance to vehicles in left lane
* relative speed of vehicle in host lane to vehicle in potential lane
* host speed

If any criteria are unsatisfactory, the host determines that the lane change is infeasible. If feasible, the host begins its execution of the lane change and commits to it, so as to be decisive.

#### Spline generation

The code then generates a spline with roughly 2 seconds of planned path. Assuming no lane change, the code generates a spline along the center of the lane. If a lane change is occuring, the spline is drawn that its end point is in the center of the target lane using a Jerk Minimizing Trajectory (code in helper file). Currently, 5 points are generated for the spline.

#### Transformation

To minimize the risk of having a function that generates two Ys for any given X, the spline is then rotated so that its end point is along the x-axis. Helper functions for rotation are present in the helpers.cpp file.

#### Speed control

A controller is created to manage speed while maintiaining jerk and acceleration criteria. The controller directly controls acceleration, incrementing based on speed and acceleration to maintain 45 mph, or the speed of the vehicle ahead. You could classify the speed controller as a finite state machine, with states throttle, brake, release throttle, and release brake. Release states move the vehicle towards 0 tangential acceleration, while brake and throttle increase the negative and positive acceleration, respectively.

An iterative process is then used to pick a point along the spline that produces an adequate speed. Once a proposed point is selected that provides a speed within 0.03 m/s of the chosen speed, the point is chosen. A correction factor is applied to the point to ensure speed chosen matches more exactly.

As all of this math is happening in the transformed coordinate space, a transformation occurs after each point is generated. Next, the kinematics of the trajectory are calculated. Finally, the transformed path is passed to the JSON file, and the process begins again.

#### Acknowledgements

I would also like to give some kudos to the VCPKG project and [this Codza blog](http://www.codza.com/blog/udacity-uws-in-visualstudio) for instructions on how to get the project running on Windows and Visual Studio. If anyone at Udacity is reading this, it would make life much easier for Windows users to have this information linked.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


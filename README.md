# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.
2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Build

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Dependencies

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

---

## Reflection

### Generating Paths

The function for finding the cost of a lane considers these inputs:

| Condition | Weight | Reason |
| --------- | ------:| ------ |
| Is it on the road? ('legal') | 2048 |
| Can the car fit without collision? | 1024 |
| Is lane blocked ahead by an object? | 8 | Prefer lanes that have open roads ahead (50m). (This is on top of the speed bonus component, which considers out 100m ahead) |
| How far right is this lane? | 0 * Lanes right of leftmost | Left lanes, in theory, are faster and less busy. The simulated traffic does not have this feature, hence 0 weight. |
| How slow is this lane? | - (Speed of slowest car (in mph) 100m ahead and 15m behind current) (default is speed limit) | The negative value awards lanes that are moving faster. If lane is empty, rewards bonus of the speed limit. |
| Does this lane lead to more options? | - 6 * (legal adjacent lanes) | Prefer middle lanes (In general, options that lead to more options are more valuable. MBA in a nutshell.) |
| Does being in this lane require a lane shift? | 3 | Threshold difference for switching lanes |

The cost of a lane is calculated for the current and adjacent lanes.

### Speed control

The car in this solution attempts to follow the speed of the car ahead of it (it looks 30m ahead), defaulting to just below the speed limit.

When changing speed, the speed limit is enforced and forward speed changes are regulated to reduce jerk.

### Room for Improvement

- Regulate rotation-based jerk
- Prevent changing lanes if changing lanes or just changed lanes. (Not in lane or has delta-d above a low threshold)
- Tried statefulness at some point, but I did not find much value in states proposed in class.

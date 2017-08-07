# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator. You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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
    
---

## Starting the project, driving over the map

Since I've been frustrated on how to initially start implementing the project I decided to start with the thing I already have: I made the car drive over
the waypoints from the highway_map. It was pretty straightforward and I've been able to implement that quickly. Next step for me was offsetting the from
the center of the road and trying to drive the car through the waypoints but trying to stay on the lane (not smoothly, quite terrible, but it worked).
The hardest part has been to drive smoothly over the track and have smooth velocity, acceleration and small Jerk. To do that I've
used simple motion physics formula in order to calculate speed, acceleration and distance. In addition I set speed limit in order the car won't be able to
drive faster than it is permitted. After deriving that data I've used Jerk formula from the lesson to get minimizing Jerk trajectory. After that I've got
point corresponding to that trajectory and interpolated them in order to get nice and smooth curve corresponding to the selected trajectory.

As a next step I decided that the car should drive in a safe manner, keeping the lane and following vehicle in front of it. 
I've found the closest car in front of our car and used the data to calculate it's speed in order to safely follow this car and applied the discussed above
approach in order to keep following the lane with respect to the speed of the car in front of us.


## Define Car Behavior States

In order to continue I've decided that some decision logic is required to I've come to the idea of adoption the lecture based approach with Finite State Machines. 
I've implemented states as enum, totally getting to the KEEP_LANE, CHANGE_LEFT, CHANGE_RIGHT, PREPARE_TO_CHANGE_RIGHT and PREPARE_TO_CHANGE_LEFT. 
However in the end I've decided to go without 2 prepare states and left only one of them called PREPARE_TO_CHANGE.

For the car in order to change the lane - the other lane should have enough space for the car to fit in with addition 
of some buffer space around it based on S coordinates. In addition if there is a car driving on the target lane behind us then our speed should correspond 
to the cars' speed in order not to collide with it and be able to safely change lane. My logic of changing the lanes also handles
the case when there are to options (left and right) for us to change the current lane: if both of lanes are free - left lane would be
the choice since in the real life it is typically 'faster' lane. 
In the end if the lane change is not possible, but desirable the car will wait for the best opportunity to change the lane, but continue keeping it until it is 
safe to perform the maneuver.

Car drives for 10 miles without any major issues:

[![10 miles drive](https://img.youtube.com/vi/_fPIWBa80TA/0.jpg)](https://youtu.be/_fPIWBa80TA)

---

## Future Improvements

My path planner is not able to handle difficult situations like slowing down in order to overcome the vehicle, e.g.: we're on the right lane, the slow vehicle is
in front of us and we'd like to overcome it, but in the center lane there is another slow vehicle. So possible solution would be to slow down, 
let the slow vehicle in the center lane pass, and after that move to the empty leftmost lane previously blocked by the slow vehicle in the middle lane.


# Path-Planning-Project

##### Author: Aparajith Sridharan
##### Date : 13.06.2021
---
### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +/-10 MPH of the 50 MPH speed limit. 
You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. 
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
Also the car should not experience total acceleration over 10 m/s^2^ and jerk that is greater than 10 m/s^3^.

### Simulator
The Term3 Simulator can be downloaded which contains the Path Planning Project from the [releases tab](#https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

["sensor_fusion"] A 2d vector of cars and then that car's  
[car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2^, also the jerk should not go over 50 m/s^3^. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

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

[aarch]: ./doc/overall_arch.jpg "Architecture"
[pred]: ./doc/object_prediction.jpg "prediction"
[JMT]: ./doc/JMT.jpg "trajectory"

# Code Documentation
## Architecture

The Path Planner architecture is as shown below.

![alt text][aarch]

As seen the path planner task is split into the following functional modules

- Map Route Generation - `Map class`
- Scene Object Prediction - `Objects class`
- Path Planning - `PathPlanner class`
- Trajectory generation - `Trajectory class`
- Cost Computation - `Cost class`

where the `PathPlanner` class is the main entry point of the planner that generates targets or goals.

### Map Class 
The `Map` class object is the first to be instantiated and the last to be destructed.  
- This module basically improves the interpolation of the waypoints in the map by using a cubic spline. 
- the cubic spline improves the accuracy of interpolation. Instead of doing a linear interpolation between two closest waypoints, 
  it does a cubic spline fitting.
- Since, it is computed once at the start for the entire route, it does not cause any performance overhead in terms of time complexity. 
- The resolution is later helpful for the Jerk minimizing trajectory generation in the trajectory planner.

### Objects Class
- The `Objects` class does a basic behaviour prediction of the scene around the Ego vehicle.  
- The class is responsible to project using the current data of the objects basically the frenet s and d , cartesian x, y , dx, dy and compute the speed, future x,y and yaw angle i.e. heading.
![alt text][pred]
- After getting the speed and filtering the frenet `d` and frenet `s` the objects that are outside the `kPredictionHorizon` which is around +/-70 m in the frenet `s` and `d` of 0 to 12m are ignored.
- using the newly filtered relevant objects the closest front and rear objects in all the lanes are identified.
- TTC (time to collision) and TTD (time to decelerate) are computed. This is essential to find free space and max possible safe speed in a lane segment.
- the free space of all the three lanes are stored and the max speed allowed in the three lane segments are stored.
- the `PathPlanner` will then use this information to generate trajectories. 

### Path Planner Class
The `PathPlanner` is responsible to generate targets or goals without thinking too much about it's viability. 
the path planner will only govern the bare minimum legal aspects of safe driving behaviour such as 
- *Traffic lane laws* : which lanes to drive in for a long duration
- *Etiquette in switching lanes* : Do not make a direct switch into a lane that would require crossing another lane a.k.a double lateral lane change.
- *respect safety distances* : this is to ensure that the ego vehicle does not end up going into a confusing traffic situation ending in a solver stalemate requiring an emergency braking maneuver.
To make it easier for implementation, The German traffic law system is used. This was chosen to avoid much research into the legal aspects as both countries USA and Germany have the same traffic side.
- Rules make lane change targets such as:
  - **Rechtsfahrgebot** - basically means prefer the right lanes to cruise/drive as a routine lane. this is the right-most lane marked as `ROUTINE` lane. Ego will prefer this lane if it is empty.
  - **Rechtsueberholverbot** - do not overtake on the right if possible. this does not include passing a slow moving vehicle travelling on the `ACCELERATION` lane.
  - `ACCELERATION` lane is the center lane, where vehicle shall accelerate quickly and reach the top speed. The EGO may prefer a `OVERTAKING` or `ROUTINE` lane depending on their free spaces.
  - `OVERTAKING` lane, which is the leftmost lane will be preferred when it is free for the `kPredictionHorizon`.

- Other targets which doesn't include a lane change maneuver include a `cruise` and `spacing` control targets. The driver set speed is the target velocity for both.
  - the `Cruise control` target is basically the vehicle in a open free space accelerating to reach `50MPH`. in this case it is `49MPH` so that the max speed is not violated.
  - the `Spacing control` target is when there is a vehicle restricting the free space of the EGO. the target velocity is set to the vehicle in the front.
- Finally, one more target named the `AEB` Automatic Emergency Braking target which will decelerate the vehicle at max deceleration to avoid a collision is added.
- A maximum of 4 targets are generated and a minimum of 3 targets are generated. 

### Trajectory Class
- Goal is to generate trajectories. nothing is validated while generation. trajectories in both frenet and cartesian are generated using the `Map` object and the `Objects` prediction data.
- Trajectory is generated using two methods.
  - **JMT - quintic polynomial solver** - jerk minimizing trajectory 
  - **frenet trajectory generation** based on `s` and `d` 
- JMT trajectory is used as the primary generator. it is resposible to generate a minimum jerk trajectory for the given target.
  - Based on solving a quintic polynomial equation as shown.
  ![traj][JMT]
- The **frenet trajectory generation** is a special case to handle emergency braking maneuver because, the JMT solver that was developed will fail to provide a solution for T = 0s.  
  see the above matrix in the picture! The T matrix will become singular! hence, it is not invertible to give a solution for a minimum jerk trajectory.
- The final step is to compute costs for all the trajectories that were generated. 
- the minimum cost is selected. if all the trajectories involve collision with predicted objects then the `AEB` trajectory is chosen.


### Cost Class
The corner stone of the trajectory generator is the `Cost` class. the `Cost` class does the following
- compute *max acceleration* and *total acceleration* of the trajectory in question.
    - governs the *max acceleration* not exceeding +/-10 ms^-2^ and *total acceleration* over all the steps in the entire trajectory of +/-10ms^-2^ 
- compute *max jerk* and *total jerk* of the trajectory in question.
    - governs the *max jerk* and *total jerk* to not exceed +/-10 ms^-3^
- compute *collision cost*. 
    - a large positive cost of 10^5^ is atleast given to collision that is compounded by trajectory point where the collision occurs.
    - a collision would straight off result in the elimination of the trajectory from selection.
    - never will a collision trajectory make it's way into the output as a selected trajectory.  
- compute *efficiency cost*.
    - this is basically the difference between free space and the `kPredictionHorizon`. The difference is between 0 and 70. less the better.
- total cost is computed after weighing the individual costs. the mandatory checks are given high cost. the less mandatory is given a lower cost. 

### Other helper classes
- **function**  a function(t) class that computes N derivatives of the given polynomial coefficients.
- **Rectangle** a class which implements SAT (separating axis theorem) to find overlapping objects to compute collision on predictions of object positions and generated trajectories to compute collision cost.
 
### Putting it all together
After a trajectory is selected. the path planner returns this trajectory so that the simulator vehicle can act.
In order to account for the simulator latency, 5 previous points from the trajectory are reused.


### References:
1. [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](#https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)
2. [Separating Axis Theorem](#https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169)
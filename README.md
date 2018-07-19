# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
The goal for this project is to navigate a car through traffic while trying to maximize its speed within the speed limit.  The car must behave in a manor which is acceptable for highway driving considering a human passenger.  This requirement puts limitiations on the behavior such as minimizing the acceleration and jerk for a comfortable ride, maintaining lane position, and most importantly not colliding with other cars.  The car must generate a trajectory that includes lane changes in an effort to maximize speed where available.  

### Repo
The original project repository can be found [here](https://github.com/udacity/CarND-Path-Planning-Project)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Using what was given
A lot of the very tricky bits to the trajectory generation were provided from the project walkthrough and I kept those the same.  The methodology given was to generate new points ahead of the current trajectory at given distances in frenet coordinate and then using the spline library to fit a smooth trajectory through the generated points.  The final points are converted to the x-y coordinates needed by the simulator.  All of the conversions between coordinate systems (frenet, cartesian, car-based cartesian) were given and I used them as-is. 

### Path Planning
The calculated trajectory used two parameters to generate that trajectory, lane and velocity.  I chose to implement a sort of finite state machine to chose the value of these parameters and thus influencing the trajectory.  I used 7 states and various logic between the transistions.  
#### States

##### Accelerate
  This state is used initialy to get up to the speed limit and whenever coming out of the deccelerate state as long as it is safe (no cars in front).  I chose to use an acceleration interval that maxed the jerk and acceleration at 8 m/s^2

##### Deccelerate
  Used anytime there is a car in front and no safe passing lanes

##### Maintain
  This is the default state where the speed doesnt change and the car maintains its lane

##### Try Change Lanes
  This is a state where the car is looking to change lanes.  It only decides it wants to change lanes when its speed is lower than a threshold which I set as 20% lower than the desired velocity, in this case it was 40mph.  From this state we check to see which lane is safe and which lane would be optimal to transition to.  It choses which lane to transition to (either right or left) based on a cost function and minimizing the value of the cost function.

##### Change Lane Right
  This is a single-cycle state transition used to increment the lane variable

##### Change Lane Left
  This is a single-cycle state transition used to decrement the lane variable

##### Changing Lanes
  This state is used to allow the lane change to finish before trying to calculate a better trajectory.  This is primarily done so that we maintain the center of a lane and dont get caught in indecision in the middle of a lane transition.

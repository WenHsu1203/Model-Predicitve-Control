# Introduction
Implementing a Model Predictive Control to drive the car around the track with a 100 ms delay as the command propagates through the system. 

__What I have: State of the Vehicle In Each Time Step__
1. __Px:__ Location x-axis
2. __Py:__ Location the y-axis
3. __Psi:__ Heading direction
4. __V:__ Velocity
5. __delta:__ Steering angle
6. __a:__ Throttle value

__What my GOALs are: Implement the MPC to Maneuver the Vehicle Around the Track__
1. Define the `timestep length(N)/duration(dt)` of the trajectory
2. Define the `cost function`, `dynamic model` and `boundaries of constraints`
3. Pass the current state to optimization solver which returns control inputs that minimize the cost. (The solver I use is called Ipopt)
4. Use `kinematic equations` to predict the states for after 100ms to handle a 100 ms latency
> The MPC setup (picture from Udacity Self-Driving Car lecture)
![Image1](https://github.com/WenHsu1203/MPC/blob/master/photos/MPC%20steps.png?raw=true)

__Cost Function__
Here are the goals I want to achieve:
1. `Cte`(cross track error) and `epsi`(error of psi) to be `0`, and `v` as high as the maximum value `100` (mile/hr)
2. Minimize the `usage of the actuator`
3. Minimize the `input gap` between sequential actuations

So my cost function looks like this:
```
cost = cte^2 + epsi^2 + (V-Vmax)^2 + delta^2 + a^2 + (delta-delta_prev)^2 + (a-a_prev)^2
*** ALL with a multiplication factor to be tuned
```

# Result
In the video, the `yellow` is a polynomial fiited to `waypoints` and the `green line` represents the x and y coordinates of the `MPC trajectory`
> Check out the video ☟
[![Video](http://img.youtube.com/vi/AjmDGNPcCFo/0.jpg)](http://www.youtube.com/watch?v=AjmDGNPcCFo "Model Predictive Control")

# Dependencies
* cmake >= 3.5
* All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

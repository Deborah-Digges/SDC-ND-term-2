# CarND-Controls-MPC

## Project Description

The purpose of this project is to develop a nonlinear model predictive controller (NMPC) to steer a car around a track in a simulator. The simulator provides a feed of values containing the position of the car, its speed and heading direction. Additionally it provides the coordinates of waypoints along a reference trajectory that the car is to follow. All coordinates are provided in a global coordinate system.

# The Vehicle Model

The vehicle model used in this project is a kinematic bicycle model. It neglects all dynamic effects such as inertia, friction and torque. The model takes changes of heading direction into account and is thus non-linear. The model used consists of the following equations

      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
Here, x,y denote the position of the car, psi the heading direction, v its velocity cte the cross-track error and epsi the orientation error. Lf is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability. The vehicle model can be found in the class FG_eval.

# Polynomial Fitting and MPC Preprocessing

All computations are performed in the vehicle coordinate system. The coordinates of waypoints in vehicle coordinates are obtained by first shifting the origin to the current poistion of the vehicle and a subsequet 2D rotation to align the x-axis with the heading direction. Therby the waypoints are obtained in the frame of the vehicle. A third order polynomial is then fitted to the waypoints. The transformation between coordinate systems is implemented in transformGlobalToLocal. The transformation used is

 X' =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
 Y' =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
where X',Y' denote coordinates in the vehicle coordinate system. Note that the initial position of the car and heading direction are always zero in this frame. Thus the state of the car in the vehicle cordinate system is

          state << 0, 0, 0, v, cte, epsi;
initially.

# Optimal Control Problem

For every state value provided by the simulator an optimal trajectory for the next N time steps is computed that minimizes a cost function. The cost function is quadratic in the cross-track error, the error in the heading direction, the difference to the reference velocity, the actuator values and the difference of actuator values in adjacent time steps. Specifically I chose parameter values here that lead to smooth driving both for slow (25mph) and fast velocities (70mph). The control problem is restricted by the vehicle model as well as actuator max and min values. The cost function can be found in FG_eval.

In the receeding horizon problem the cost functio is minimized at each time step, but only the actuations corresponding to the first time step are sent to the simulator. At the next time step the entire optimal control problem is solved again.

# Model Predictive Control with Latency

We account for latency by assuming the current car drifts at the current speed, heading, and rate of turn for the entire interval forward. These become the initial state for our model. Our algorithm then selects an optimal sequence of steering and throttle adjustments, 100 times a second, for that time forward. This is equivalent to looking ahead while you're driving, realizing you can't do that much about what's immediately in front of you at highway speeds. Your decisions now affect your location, heading and speed a few feet in front of you, not where you are at the current instant.




# Timestep Length and Elapsed Duration (N & dt)

The time T=N dt defines the prediction horizon. Short prediction horizons lead to more responsive controlers, but are less accurate and can suffer from instabilities when chosen too short. Long prediction horizons generally lead to smoother controls. For a given prediction horizon shorter time steps dt imply more accurate controls but also require a larger NMPC problem to be solved, thus increasing latency.

Here I chose values of N and dt such that drives the car smoothly around the track for slow velocities of about 25mph all the way up to about 70mph. The values are N=12 and dt=0.05. Note that the 100ms = 2*dt latency imply that the controls of the first two time steps are not used in the optimization. They are frozen to the values of the previous actuations, like so

# Cost Function Parameters

The cost of a trajectory of length N is computed as follows
```
   Cost  = Sum_i cte(i)^2 
              + epsi(i)^2 
              + (v(i)-v_ref)^2 + delta(i)^2 
              + 10 a(i)^2 
              + 600 [delta(i+1)-delta(i)] 
              + [a(i+1)-a(i)]
where the increased weight on steering changes between adjacent time intervalls is the most important in order to arrive at smooth trajectories.
```
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

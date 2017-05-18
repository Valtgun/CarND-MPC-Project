# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
---
## The model
### State
State is received from simulator, with the following parameters:
x, y - the vehicle coordinates
psi - vehicle orientation
speed - vehicle speed
x, y, and psi are used to convert waypoint coordiates from map space to vehicle space. Vehicle space is used for path planning and polynomial fitiing.

### Actuators
There are two actuators returned by the solver.
delta (in file mpc.cpp) which is steering actuator - solver returns values which should be inverted to have correct steering angle.
a (in file mpc.cpp) which is throttle actuator with positive values meaning acceleration and negative meaning braking.
The values are returned by solver and passed ot the simulator.

### Update equations
Update equations were used as described in the classroom. MPC.cpp lines 117-124.
Kinematic model was used.
For example, x coordinate calculation used equation from classroom:
x​t+1​​=x​t​​+v​t​​∗cos(ψ​t​​)∗dt

### Error calculation
One of the time consuming and most important adjustments were multipliers for error calculation. I have used the example from the lectures and adjusted the multipliers for the car to be able to drive around the track.
The values in the submission were found by manual tuning and mostly in the approach that values were changes in scale of 10x to observe the behavior and then more finely adjusted.

Following multipliers (weights) were used (mpc.cpp lines 54-82):
v_mult = 1; for difference with target velocity, if this value was increased then solver returned very erratic results and if it was decreased below 1 then solver did not accelerate the car, which lead to keeping this value equal to 1.

cte_mult = 1000; for distance from path, this was one of the most significant error components to keep car close to the path.

epsi_mult = 500; for angle error, it was not as significant as distance from path above, but still one of most important components. In the value was significantly lowered, then car did not follow the corners properly and run to outside of the road.

Next parameters were used to minimize the actuator input and it provided smoother driving behavior. In case those parameters are set too high then the car can not take sharper turns properly. And lower weights mean that vehicle path will be more erratic ad the actuator inputs are 'easy' to change.
act_steer_mult = 50; for steering
act_vel_mult = 25; for acceleration

Final two parameters were used to minimize difference between actuator in sequential timesteps. In my testing 5x the actuator input provided reasonable results. Which means that solver would try to find the better initial steering and throttle controls and try to minimize the adjustments between timesteps.
delta_act_steer_mult = 250; for steering
delta_act_vel_mult = 125; for acceleration

### Target speed
In the MPC model one of the parameters that were adjusted was ref_v (mpc.cpp line 17) which determine the velocity that vehicle would try to attain.
In my case I set it to the 100 mph.
However, based on the solver results in case of straight road segments it would attain around 75%-85% of the target speed.
As there are steering adjustments required, the speed is sacrificed and the target speed of 100 mph is not achieved.

## Timestep Length and Frequency
Those parameters were related between them, if the timestep length N was increased then frequency was decreased.
### Frequency - dt
This was more important parameter to adjust than timestep length.
If the frequency was too long, then the first actuation was relatively 'far' then algorithm resulted in 'smooth' path which did not react sufficiently, especially in the sharper corners.
If the frequency was short, then first actuation was very close to the car, which lead to larger control inputs and car began to oscillate around the waypoint path.

### Timestep Length - N
Based on the adjusted frequency, the number of time-steps were adjusted. In this case it was 10 time-steps with 0.15 second interval. Which means that algorithm was taking in account what would happen with the car in next 1.5 second.
If the parameter was too large then calculations lost accuracy, especially in the segments where are changing road curvature.

### Considerations for improvement
I observed that it would be possible to improve the approach if above parameters would not be fixed, but would dynamically adjust to vehicle's speed.
In case the vehicles speed increases then calculations should be performed more frequently and vice versa.

## Polynomial Fitting and MPC Preprocessing
### Polynomial Fitting
3rd degree polynomial fitting was used by utilizing the provided functions.

### MPC Preprocessing
In the code, there was coordinates preprocessed to convert them from the map space to the vehicle space, based on the vehicle coordinated and orientation.

## Model Predictive Control with Latency
### How it is dealt with latency
Vehicle driving behavior in response to latency was adjusted by choosing the matching frequency.
In addition to that **very important** point regarding latency was processing speed. In case the processing speed was not performing and took long time to provide the results, vehicle controls were delayed. Which lead to more driving errors. There is fine balance between accuracy and processing speed.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from [here](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# CarND-MPC-Control
This is a Self-Driving Car Engineer Nanodegree Program completed by Michael chen.This program implements a MPC controllor for controling a vehichle.


## Build and Run

1. clone this repository to your local machine.
2. implement below order step by step.  
>  mkdir build && cd build  
>  cmake .. && make  
>  ./mpc   
3. open the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).and Select "the Project 5:MPC controllor".

##  Discussion about implementation

### Model discription
The aim of the MPC control is to minimize the two state values: cte(cross track error) and eψ(error of the head direction).
The MPC model will build a model which transfer the problem to minimize a cost value.
The cost value have three basic parts and five additional parts(cost constraints):  
* cte(cross track error)
* eψ(error of the head direction)
* velocity error(the error between current velocity and the reference velocity)
* control input(actuation of sterring):δ
* control input(actuation of accelerating/deccelerating):a
* error between δ of current step and δ of next step
* error between a of current step and a of next step
* error between cte of current step and cte of next step  

when optimize the cost values,we need set the lower and upper limits for the variables.
all varialbels are shown as below:
* x - coordinates at each disrete point
* y - coordinates at each disrete point
* psi - the head direction angle at each disrete point
* v - the velocity of the vehicle at each disrete point
* cte - cross track error at each disrete point
* epsi - error of head direction at each disrete point
* delta - control input(actuation of sterring)
* a - control input(actuation of accelerating/deccelerating)

the main limits are as follows:
* The upper and lower limits of delta are set to 25 and -25
* Acceleration/decceleration upper and lower limits are set to 1 and -1
* Lower and upper limits for the constraints are set to 0

### Discussion Timestep Length and Elapsed Duration (N & dt)
MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".A good approach to setting N, dt, and T is to first determine a reasonable range for T and then tune dt and N appropriately.  
Considering the vehicle's motion,Assuming the vehicle can reach the speed 100 mph, namly 44.7 m/s.If we want to predict the actuation in 30-50 meter away,then we can set the T is 30/44.7-50/44.7(0.67-1.12 second).Because the vehicle's speed is mostly under 100 mph,the predicted time can be set longer than 1.12s.In my solution,I set the T to be 1.25s.  
The smaller the dt is ,The more accuracy the approximation is,but if dt is too small,the optimization need more resource to calculate and the running time will be long,it will not meet the real-time requirement.It is a contradiction.Here I choose N is 25,dt is 0.05s.They can reach a good performance. 
### Polynomial Fitting
In MPC Algrithm, we approximate a continuous reference trajectory by fitting a polynomial curve.In my solution,I use a 3rd order polynomial curve to fit the trajectory waypoints.First,I define a function named 'transformMapToVehicle',which can transfer the waypoint from global coordinates to vehicle cooridinates. then define a function named 'polyfit',which can fit the curve by a 3rd polynomial curve.Then Using this curve,I can calculate cte and eψ.Please see the code details in main.cpp.
### Dealing with latency
In the solution,when sending the control command to the vehicle,we will sleep for 100 milliseconds,this can simulate the latency in a real car.
This means that our command will effect the car 100 milliseconds later than the current state. If the velocity of the vehicle is low,it will not have a big influnce.But if the velocity is high,it should be paid more attention on.  
In my code,I use two method to solve this problem.The first method is that when I make the prediction control,I predict the state of 100 millisecond later at first,then I implement the MPC Algrithm. Another method is that I calculate the derivation in front 6m,this value will be the factor to calculate a value reduce the reference velocity,when vehicle meet a turn,the reference velocity can be reduced by the factor.In my code,it is as follows:  
> fg[0] += CppAD::pow(vars[v_start + t] - ref_v/(1.0+5 * CppAD::abs(coeffs[1]+12 * coeffs[2]+108 * coeffs[3])), 2);   

'coeffs[1]+12 * coeffs[2]+108 * coeffs[3]' is the value of the derivation in front 6m. '5.0'is a amplified factor.and 1.0 is used to avoid that ref_v divided by zero.  
After running in the simulator,it shows that The highest speed of the vehicle can reach nearly 90 mph.and the vehicle can always keep on the center of the track.  

## Dependencies

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



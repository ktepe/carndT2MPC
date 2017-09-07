# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program



## Kemal Tepe, ketepe@gmail.com

## The Model
* The state, actuators and update equations. 

The kinematic model given below is implemented in the solver:

```math 
x(t+1)=x(t) - v(t)cos(phi(t))d_t;
y(t+1) = y(t) - v(t)sin(phi(t))d_t;
phi(t+1) = phi(t) - (v(t) delta(t)/Lf d_t;
v(t+1)=v(t)+a(t)d_t
cte(t+1) = f(x(t))-y(t)+v(t)sin(ephi(t))d_t;
ephi(t+1) = epsi(t)- epsi_{dest}(t)+v(t) delta(t)/Lf d_t ;
```
where ``Lf`` is turning radius constant, ``d_t`` sampling interval, ``a(t)`` acceleration (throttle), and ``delta(t)`` steering angle.  Variable ``a(t)`` and ``delta(t)`` are actuations. 

## Timestep Length and Elapsed Duration (N & dt)
* The reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Number of options for N and dt are tried. As suggested in the project manual, (Nxdt) is selected 1.5 to 2 seconds. For dt, I tried 0.05 and 0.1 seconds. Selecting dt as 0.05 requires at least an N of 30-40 timesteps and increases the computational complexity of solver. Finer dt resolution can increase the control accuracy but extra computations need to be avoided as well. That is why in the final implementation dt is selected as 0.1. This also alligns well with the latency in applying the control.  After setting dt to 0.1 sec, number of different values for N is tried. N varied from 10 to 20. Although there were not a significant performance difference among the number between 10-20, in the final implementation I left N as 20 in order to provide longer control horizon, selecting N 20 seemed to handle sharp curves better. 

* Update (after review)
I reduced to N=12 and the vehicle had a better handling since the predictions were fitting to the waypoints better.


## Polynomial Fitting and MPC Preprocessing
* A polynomial is fitted to waypoints.

The solver optimizes acceleration values and steering angle for given cost function. The solver first fits ``x(t)`` and ``y(t)`` values to the 3rd order polynomial which fits to given waypoints. The cost function is implemented in the solver is given below:

```C++
  //now cost function Similar to the lectures
    for (int t = 0; t < N; t++) {
      fg[0] += 20*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 3.5*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 0.2*CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    // minimize acutations
    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 2*CppAD::pow(vars[a_start + t], 2);
    }
		// Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 700*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 30*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      // slow down the vehicle when the steering is large 			
      fg[0] += 1000*CppAD::pow(vars[a_start + t]* vars[delta_start+t], 2);	
      
    }
```

* Update (after review)

The weights of the cost function parameters are manually optimized for reference speed of 50 mph or less. I also put a cost on acceleration and large steering. This allowed to car to slow down at sharp turns and stabilized the car. With that, the car can reach top speed of 35 mph on the track. 




## Model Predictive Control with Latency
* Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

* Update (after review)

Since there is a roughly 0.1 sec delay between applying actuation values and the vehicle position, I used the approach of providing the estimated vehicle's state vector at 0.1 sec to the solver. 
```C++
//calculate what would the predicted state in 0.1 sec. to compensate the delay
	double dt=0.1;
	double x_pre=v*dt;
	double y_pre=0;
	double psi_pre= v * -(steering_angle / 2.67) * dt;
	double v_pre=v+v*throttle*dt;
	double cte_pre = cte + v * sin(epsi) * dt;
	double epsi_pre= epsi + v * -(steering_angle / 2.67) * dt;	
						
//	state_vec << 0, 0, 0, v, cte, epsi;
	state_vec << x_pre, y_pre, psi_pre, v_pre, cte_pre, epsi_pre; 
```



## Conclusions:

* The controller works upto speeds 35-40 mph on the track without any problem. With some more time and cost weight adjustments, the controller can handle higher speeds.

* Model predictive control lecture and codes provided there were very useful. I also heavily used Forum discussions to implement and finetune the controller.


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
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


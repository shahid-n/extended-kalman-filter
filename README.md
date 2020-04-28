# Extended Kalman Filter Project
**Self-Driving Car Engineer Nanodegree Program**

## EKF Project Goals 

This project entails the creation of an extended Kalman filter to fuse radar and lidar measurements, and to output filtered next-state estimates whilst also iteratively updating the state variables based on the current measurements, which are corrupted by noise. The goals of this project are twofold.

1. Track the motion of a single object in the _xy_-plane in real time, based on noisy radar and lidar measurements
2. Keep the overall root-mean-squared error (RMSE) -- for the example run using Dataset 1 -- below the prescribed thresholds of `0.11 m` for both position estimates, and `0.52 m/s` for the estimated velocities, respectively

Additionally, the source files, output and results of this project must also meet the criteria laid out in the linked [project rubric](https://review.udacity.com/#!/rubrics/748/view).

[//]: # (Image References)

[data1]: ./output/sim_output_datatset_1.png "Output from Dataset1"
[data2]: ./output/sim_output_datatset_2.png "Output from Dataset2"
[bug]: ./output/bug_heading.png "Bug due to unrestricted heading angle tracking error"

---

## Repository Overview

In order to succesfully compile the source code and run the executable to process the measurement data and communicate with the simulator, the following files needed to be filled in with the correct code.
1. [FusionEKF.cpp](./src/FusionEKF.cpp) which initialises the EKF state variables and calls both the prediction and update methods of the `ekf_` object
2. [kalman_filter.cpp](./src/kalman_filter.cpp) which defines the `KalmanFilter` class, of which the `ekf_` object mentioned above is an instance, and all associated prediction and update methods, whose equations had to be filled in
3. [tools.cpp](./src/tools.cpp) which contains supporting methods to calculate the Jacobian matrix for the radar update equations, and also to compute the RMSE during simulation

This project was compiled and built using the default [CMakeLists.txt](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/master/CMakeLists.txt) file provided in the project starter repository.

## Results

Following a few iterations of debugging and recompiling, the executable was eventually able to successfully communicate with the simulator and generate its state predictions and updates, along with the RMSE values.

The results corresponding to `Dataset 1` are shown below, and it is clear from the screenshot that the RMSE metrics from the rubric have been met.
![alt text][data1]

Similarly, the following screenshot depicting simulation results corresponding to `Dataset 2` is also in compliance with the RMSE criteria stipulated in the rubric.
![alt text][data2]

That being said, a tricky numerical bug was the root cause for large tracking errors when the object was looping back close to the origin whilst replaying `Dataset 1`; this bug fix is discussed in more detail in the next section.

## Project Challenges

### 1. Compile time and run time errors

Whilst it was relatively straightforward to initially eliminate all syntax errors through proper declaration of all the variables and careful review of calls to member functions and their associated prototype definitions -- not to mention learning to work with the specialised `VectorXd` and `MatrixXd` data types -- it was a little bit more challenging to ascertain the root cause for a rather cryptic run-time error which cropped up even after the project was compiled successfully. An excerpt of the error message is shown below.
```
Assertion 'index > = 0 & & index < size()' failed. Aborted (core dumped)
```

It turned out this error was occurring due to some variables that had not been properly initialised within the `ekf_` object -- once all vector and matrix elements were properly assigned initial values, this problem was resolved.

### 2. Numerical bug during simulation

A more intractable problem occurred during simulation, especially with `Dataset 1` -- the RMSE values were growing large too quickly, and there was a clearly distinguishable discontinuous jump during the overlapping leg of the figure eight manoeuvre, complete with a 180-degree flip in orientation from one instant to the next, as depicted in the screenshot below.
![alt text][bug]

The root cause for this bug is apparent from the [documentation](https://en.cppreference.com/w/cpp/numeric/math/atan2) of the `atan2()` function -- specifically, whenever the trajectory is either: a) parallel or tangent to the _x_-axis and intersects with the _y_, or b) parallel or tangent to the _y_-axis and simultaneously intersects with the _x_-axis, the lack of numerical resolution in one or both arguments to this function can lead to a sudden jump between 0 and +/-_&pi;_ radians in case a), or a discontinuous and instantaneous jump between +/-_&pi;_/2 radians in case b).

This issue is further exacerbated by the difference calculation _y_(1) = _z_(1) - _&phi;_, namely the error in the heading angle, which could see instantaneous jumps on the order of _&pi;_ if this issue were not addressed, and this is precisely what was happening as shown in the screen capture above.

This numerical bug was fixed by adding the following code in [lines 63 to 68 of kalman_filter.cpp](https://github.com/shahid-n/extended-kalman-filter/blob/master/src/kalman_filter.cpp#L63):
```
  while (y(1) < -M_PI) {
    y(1) += 2*M_PI;
  }
  while (y(1) > M_PI) {
    y(1) -= 2*M_PI;
  }
```

This code works because in a real world physical process such as a moving car or object, the changes in position as well as orientation are continuous in time (not to mention "smooth" under a suitable mathematical definition of the term) -- consequently, even upon discretisation via sampling, the expected successive changes in the sequence of position or heading errors are relatively small. Indeed, for a sufficiently small sampling interval, it is virtually impossible for an object such as a car to instantaneously flip its heading by a full _&pi;_ radians. Therefore, the process of iteratively adding or subtracting 2 _&pi;_ as appropriate, until the heading error _y_(1) is always restricted within the interval [-_&pi;_, _&pi;_] and never outside it, successfully addresses this numerical issue.

## Concluding Remarks

This project was a good introduction to real time sensor fusion and state estimation in the presence of additive white, Gaussian distributed measurement noise (AWGN). The concepts and techniques introduced here are invaluable in practical applications of signal processing and control algorithms in embedded systems.

### Next Steps

A natural extension of this project would be to seek out more state of the art techniques based on the theory of nonlinear observers and robust nonlinear control of multi-input, multi-output (MIMO) systems.

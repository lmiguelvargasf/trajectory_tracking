# trajectory_tracking
This is a ROS application to test trajectory tracking algorithms for mobile robots using Gazebo as simulator.
The robot that has been used for this project is [Turtlebot 2][1] which is an affordable mobile robot widely used for research.
People who are interested can test these algorithms in a real robot.

The algorithms that have been used so far are:
* Numerical method controller using Euler's approximation.
* PID controller

These algorithms have been tested, and are properly working for the following trajectories:
* Linear Trajectory
* Circular Trajectory
* Squared Trajectory

### Trajectories
Three trajectories have been used to test each controller.

#### Linear Trajectory
This trajectory has been defined by the following parametric equations:

<img src="images/equations/linear_trajectory.png" alt="Linear Trajectory" width="300">

#### Circular Trajectory
This trajectory has been defined by the following parametric equations:

<img src="images/equations/circular_trajectory.png" alt="Circular Trajectory" width="300">

#### Squared Trajectory
This trajectory has been defined by the following equations:

<img src="images/equations/squared_trajectory.png" alt="Squared Trajectory" width="300">


### Experiments and Results
The performance of both controllers was excellent, and this is shown in the obtained results.
For each experiment a video and its corresponding results are shown. In order to run the
simulation as fast as possible for Gazebo, the parameter **_real_time_update_rate_** has been
set to 0.00000 in **_worlds/room.world_**.

#### Euler Method Controller

##### Linear Trajectory

**Video:** [Euler: Linear Trajectory Test][2]

<img src="images/results/euler/linear/linear_trajectory_euler_x_y.png" alt="Results for x and y" width="600">
<img src="images/results/euler/linear/linear_trajectory_euler_theta_trajectory.png" alt="Results for theta and trajectory" width="600">


##### Circular Trajectory

##### Squared Trajectory

#### PID Controller

##### Linear Trajectory

##### Circular Trajectory

##### Squared Trajectory


[1]: http://www.turtlebot.com/
[2]: https://www.google.com/

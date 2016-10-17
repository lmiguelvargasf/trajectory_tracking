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

#### Squared Trajectory

### Results

#### Euler Method Controller

##### Linear Trajectory

##### Circular Trajectory

##### Squared Trajectory

#### PID Controller

##### Linear Trajectory

##### Circular Trajectory

##### Squared Trajectory

[1]: http://www.turtlebot.com/

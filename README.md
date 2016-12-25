# trajectory_tracking
This is a ROS application to test trajectory tracking algorithms for mobile robots using Gazebo as simulator.
The robot that has been used for this project is [Turtlebot 2][1] which is an affordable mobile robot widely used for
research.

Algorithms that have been used and are working properly so far:
* Numerical method controller using Euler's approximation.
* PID controller

These algorithms have been tested, and are properly working for the following trajectories:
* Linear Trajectory
* Circular Trajectory
* Squared Trajectory

## Installation
Before cloning this repository, you should install ROS and Gazebo.
It is highly recommended to use Ubuntu as OS. I also suggest installing
Gazebo before ROS in order to avoid Gazebo version conflicts.

### Installing Gazebo
In order to install Gazebo, you can follow the instructions provided in
[this tutorial][9], or you can just execute this command line:

```
$ curl -ssL http://get.gazebosim.org | sh
```

### Installing ROS
The ROS version used in this project is `kinect`. In order to install
ROS, you can follow the instruction in [this tutorial][8], or you can
execute the following command lines:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo rosdep init
$ rosdep update
```

Finally you should source the enviroment variables to avoid doing it
every time you want to use ROS:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

You could optionally install `rosinstall` in order to download packages
easily:

```
$ sudo apt-get install python-rosinstall
```

### Cloning repository

Once you have installed ROS and Gazebo, you can clone this repository:

```
$ git clone https://github.com/bit0001/trajectory_tracking.git
```

## Usage
First, you need to source the workspace, and execute a launch file:

```
$ source turtlebot_ws/devel/setup.bash
$ roslaunch trajectory_tracking turtlebot_world.launch
```

This will open Gazebo in a world where a turtlebot is shown in the middle of a room.
Now, open up a new terminal, source again the workspace, and run the file `trajectory_tracking twist.py`:

```
$ source turtlebot_ws/devel/setup.bash
$ rosrun trajectory_tracking twist.py cmd_vel:=cmd_vel_mux/input/teleop
```

Finally, to run an experiment, open a new terminal, source the workspace and run the file `control.py`:
```
$ source turtlebot_ws/devel/setup.bash
$ rosrun trajectory_tracking control.py
```

By default, turtlebot will follow a linear trajectory using the numerical method controller, but you can change it
by changing the values of `TRAJECTORY` and `CONTROLLER` constants inside `src/constants.py`.
The possible values that `CONTROLLER` can take are: `'pid'` and `'euler'`, while the possible values of `TRAJECTORY` are
`'linear'`, `'circular'`, and `'squared'`.


## Trajectories
Three trajectories have been used to test each controller.

#### Linear Trajectory
This trajectory has been defined by the following parametric equations:

<img src="images/equations/linear_trajectory.png" alt="Linear Trajectory" width="190">

where (x, y) represent the position of the robot at time t, v_x and v_y are the linear velocities for the x and y axes,
and x_0 and y_0 are the initial values for x and y positions.

#### Circular Trajectory
This trajectory has been defined by the following parametric equations:

<img src="images/equations/circular_trajectory.png" alt="Circular Trajectory" width="265">

where (x, y) represent the position of the robot at time t, v is the linear velocity of the robot,
and x_0 and y_0 are the initial values for x and y positions, R represents the radius of the circle, and T the time that
the robot takes in order to complete the circular trajectory.

#### Squared Trajectory
This trajectory has been defined by the following equations:

<img src="images/equations/squared_trajectory.png" alt="Squared Trajectory" width="550">

where (x, y) represent the position of the robot at time t, v is the linear velocity of the robot,
and x_0 and y_0 are the initial values for x and y positions, s represents the side of the square, and T the time that
the robot takes in order to complete the circular trajectory.


### Experiments and Results
The performance of both controllers was excellent, and this is shown in the obtained results.
For each experiment a video and its corresponding results are shown. In order to run the
simulation as fast as possible for Gazebo, the parameter **_real_time_update_rate_** has been
set to 0.00000 in **_worlds/room.world_**.

#### Euler Method Controller - Linear Trajectory

**Video:** [Euler: Linear Trajectory Test][2]

<img src="images/results/euler/linear/linear_trajectory_euler_x_y.png" alt="Results for x and y" width="1000">
<img src="images/results/euler/linear/linear_trajectory_euler_theta_trajectory.png" alt="Results for theta and trajectory" width="1000">


#### Euler Method Controller - Circular Trajectory

**Video:** [Euler: Circular Trajectory Test][3]

<img src="images/results/euler/circular/circular_trajectory_euler_x_y.png" alt="Results for x and y" width="1000">
<img src="images/results/euler/circular/circular_trajectory_euler_theta_trajectory.png" alt="Results for theta and trajectory" width="1000">

#### Euler Method Controller - Squared Trajectory

**Video:** [Euler: Squared Trajectory Test][4]

<img src="images/results/euler/squared/squared_trajectory_euler_x_y.png" alt="Results for x and y" width="1000">
<img src="images/results/euler/squared/squared_trajectory_euler_theta_trajectory.png" alt="Results for theta and trajectory" width="1000">

#### PID Controller

#### PID Controller - Linear Trajectory

**Video:** [PID: Linear Trajectory Test][5]

<img src="images/results/pid/linear/linear_trajectory_pid_x_y.png" alt="Results for x and y" width="1000">
<img src="images/results/pid/linear/linear_trajectory_pid_theta_trajectory.png" alt="Results for theta and trajectory" width="1000">

#### PID Controller - Circular Trajectory

**Video:** [PID: Circular Trajectory Test][6]

<img src="images/results/pid/circular/circular_trajectory_pid_x_y.png" alt="Results for x and y" width="1000">
<img src="images/results/pid/circular/circular_trajectory_pid_theta_trajectory.png" alt="Results for theta and trajectory" width="1000">

#### PID Controller - Squared Trajectory

**Video:** [PID: Squared Trajectory Test][7]

<img src="images/results/pid/squared/squared_trajectory_pid_x_y.png" alt="Results for x and y" width="1000">
<img src="images/results/pid/squared/squared_trajectory_pid_theta_trajectory.png" alt="Results for theta and trajectory" width="1000">

[1]: http://www.turtlebot.com/
[2]: https://www.youtube.com/watch?v=WLiILRsv9n4
[3]: https://www.youtube.com/watch?v=8eaEPQzavvk
[4]: https://www.youtube.com/watch?v=M4-E_W8IbYI
[5]: https://www.youtube.com/watch?v=3AXAvi5Tdq8
[6]: https://www.youtube.com/watch?v=ZDh1iNf1wvU
[7]: https://www.youtube.com/watch?v=nGBi5b4sPTA
[8]: http://wiki.ros.org/ROS/Installation
[9]: http://gazebosim.org/tutorials?cat=install

# iarc_trajectory_estimation

Trajectory Generation package for the official IARC Mission 9 statement.
This package generates linear trajectory for MAVs (specifically rotary wings) by utilizing Linear Polynomial Optimization.
This idea is already implemented in the following repository and thus used here:-

[ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation): Polynomial Optimization to generate trajectories (especially for rotary wing MAVs)

## Installation

1- To get all the latest changes, do the following:-

``` 
$ cd iarc_ws/src
$ git pull origin master
```

2- Install Dependencies(if you haven't done the steps of README.md of IARC2020, first execute them):-

``` 
$ wstool init
$ wstool merge IARC2020/iarc_trajectory_estimation/install/install_ssh.rosinstall
$ wstool update
```

3- Build the package

``` 
$ catkin build 
$ source ~/iarc_ws/devel/setup.bash
```

5- Launch the node

``` 
$ roslaunch iarc_trajectory_estimation default.launch
```

If everything is fine till now, you must be seeing a trajectory from the start point of MAV to first pylon.

6- To make the firefly go to the first pylon following the trajectory, do :-

``` 
$ rosservice call /firefly/command "{}"
```

You will see the firefly following the trajectory.

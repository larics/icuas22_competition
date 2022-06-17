#!/bin/bash

rostopic pub /red/position_hold/trajectory trajectory_msgs/MultiDOFJointTrajectoryPoint "transforms:
- translation:
    x: $1
    y: $2
    z: 1.5
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
velocities:
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
accelerations:
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
time_from_start:
  secs: 0
  nsecs: 0" 

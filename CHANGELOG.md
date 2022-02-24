
# Change Log

## [1.0.5] - 25-02-2022

### Added
- ```startup/challenge/custom_config/position_control_thrust.yaml``` - Position parameters used for thrust control

### Changed
- ```startup/challenge/session.yml``` - Use the new position control parameter set where thrust is regarded as thrust, and not climb rate.

## [1.0.4] - 19-02-2022

### Added
- ```Dockerfile.focal-nogpu``` - A Dockerfile for hardware without a dedicated graphics card.

### Changed
- ```run_docker.sh``` - Disable ```--gpus all``` flag if running an image that does not use a dedicated GPU.

## [1.0.3] - 15-02-2022

### Changed
- Updated ```startup/challenge/session.yml``` to start the Ardupilot firmware with disabled GPS sensor
- Updated ```startup/challenge/session.yml``` to initialize offboard control with ```/red/odometry``` Gazebo feedback

### Added
- ```arducopter_nogps``` - Ardupilot parameters with disabled GPS sensor

## [1.0.2] - 09-02-2022

### Changed
- Updated ```startup/challenge/session.yml``` to launch world with improved lighting

### Added
- ```well_lit.world``` - world with improved lighting
- ```well_lit_world.launch``` - launch file for world with improved lighting

## [1.0.1] - 03-02-2022

### Changed
- Add [larics/storm_gazebo_ros_magnet](https://github.com/larics/storm_gazebo_ros_magnet.git) ROS package to all Dockerfiles
- Update ```startup/challenge/session.yml``` to spawn the UAV with a magnetic gripper
- Update ```startup/challenge/session.yml``` to spawn the magnetic ball after UAV takeoff

### Added
- ```spawn_ball_at_uav.py``` - controls spawning of the magnetic ball 
- ```spawn_ball.launch``` - spawns the magnetic ball

## [1.0.0] - 01-02-2022
 
### Added
   
- Added initial ICUAS 2022 challenge setup

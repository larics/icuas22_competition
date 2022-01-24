# ICUAS 2022 UAV Competition
The main repository for the ICUAS 2022 UAV competition.

## Install

You can either manually install the UAV software stack by following 
[uav_ros_simulation](https://github.com/larics/uav_ros_simulation) instruction or simply 
use Docker insted.

To install Docker on your system execute the following command:
```
curl https://raw.githubusercontent.com/larics/uav_ros_simulation/main/installation/dependencies/docker.sh | bash
```

## Build

You can either manually build all the packages on your system using the ```catkin build``` command.

Alternatively, to build the ICUAS2022 Competition solution image please execute the following command:
```
./docker_build.sh
```

## Startup

To automatically start and setup challenges navigate to the one of the three folders in ```startup``` and run:
```
./start.sh
```
This should automatically setup and start the respective challenge, as well as run your code.

* Commands that run your challenge solution (rosrun, roslaunch etc.) should be placed in the respective ```session.yml``` file.
* Software configuration specific for each challenge should be placed in the respective ```custom_config``` folders.

*NOTE* If you are unfamiliar with the Docker or Tmux commands please check out this [quick-start guide](https://github.com/larics/uav_ros_simulation/blob/main/HOWTO.md).

*NOTE* If you chose to run the challenge inside the docker environment, please run the container first using:
```
./docker_run.sh
```

## Challenge 1

TODO: Describe challenge 1, add some pictures, etc.

## Challenge 2

TODO: Describe challenge 2, add some pictures, etc.

## Challenge 3

TODO: Describe challenge 3, add some pictures, etc.
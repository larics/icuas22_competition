#!/bin/bash

# Challenge setup (maybe not relevant for arena)
if [[ -z "${ARENA_TYPE}" ]]
  then 
    export ARENA_TYPE=1
  fi;
  if [[ $ARENA_TYPE -ge 6 ||  $ARENA_TYPE -le 0 ]]
  then
    echo "arena argument should be 1, 2 or 3. Defaulting to 1."
    export ARENA_TYPE=1
  fi;
  if [ $ARENA_TYPE -eq 1 ]
  then
    export TILE_X=12.5
    export TILE_Y=-3.0
    export TILE_Z=2.0
    export TILE_YAW=3.1415926
  elif [ $ARENA_TYPE -eq 2 ]
  then
    export TILE_X=9.0
    export TILE_Y=-7.5
    export TILE_Z=2.5
    export TILE_YAW=1.57079632679
  elif [ $ARENA_TYPE -ge 3 ]
  then
    export TILE_X=7.5
    export TILE_Y=7.5
    export TILE_Z=3.5
    export TILE_YAW=-1.57079632679
  fi;

# Global Planner parameters
export ABSOLUTE_CONFIG=true
export MAP_CONFIG=$(rospack find icuas22_competition)/config/global_planner.yaml
export TRAJ_CONFIG=$(rospack find icuas22_competition)/config/global_planner.yaml
export STATE_VALIDITY_CONFIG=$(rospack find icuas22_competition)/config/global_planner.yaml
export PATH_PLANNER_CONFIG=$(rospack find icuas22_competition)/config/global_planner.yaml
export MODEL_CORRECTION_CONFIG=$(rospack find larics_motion_planning)/config/model_correction_config_example.yaml
export OCTOMAP_FILE=$(pwd)/custom_config/icuas2022_arena_latest.binvox.bt

# Optitrack parameters
export OBJECT_NAME=hawk2
export ODOM_TOPIC=/$OBJECT_NAME/vrpn_client/estimated_odometry
export OPTITRACK_IP=192.168.1.50

# Pixhawk
export UAV_NAMESPACE=red
export PIX_SYM=/dev/ttyUSB_px4:921600
export MAP_FRAME=optitrack

# Camera parameters
export CAMERA_LINK=$UAV_NAMESPACE/camera
export BASE_LINK=$UAV_NAMESPACE/base_link

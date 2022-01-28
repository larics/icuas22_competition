#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "Running Docker Container"

# Get distro of the built image
distro=$(docker images icuas22_competition | tail -n1 | awk '{print $2}')
run_args=""

for (( i=1; i<=$#; i++));
do
  param="${!i}"

  if [ $param == "--bionic" ]; then
    distro="bionic"
  fi

  if [ $param == "--focal" ]; then
    distro="focal"
  fi

  if [ $param == "--run-args" ]; then
    j=$((i+1))
    run_args="${!j}"
  fi

done

echo "Building for $distro"

docker run \
  $run_args \
  -it \
  --network host \
  --privileged \
  --gpus all \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="XAUTHORITY=${XAUTH}" \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name icuas22_competition_$distro \
  icuas22_competition:$distro \
  /bin/bash
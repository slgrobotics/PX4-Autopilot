#!/bin/bash

export SITL_GAZEBO=../Tools/simulation/gz

set -x
#cp default.sdf $SITL_GAZEBO/worlds/default.sdf
cp lawn.sdf $SITL_GAZEBO/worlds/lawn.sdf
#cp $SITL_GAZEBO/worlds/default.sdf .

ls -al $SITL_GAZEBO/worlds/.
ls -al
set +x


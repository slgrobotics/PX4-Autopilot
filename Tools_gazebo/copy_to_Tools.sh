#!/bin/bash

export SITL_GAZEBO=../Tools/simulation/gazebo-classic/sitl_gazebo-classic

set -x
rm $SITL_GAZEBO/models/r1_rover/r1_rover.sdf.jinja
touch r1_rover.sdf.jinja
ln r1_rover.sdf.jinja $SITL_GAZEBO/models/r1_rover/.

rm $SITL_GAZEBO/worlds/empty.world
ln empty.world $SITL_GAZEBO/worlds/.

ls -al $SITL_GAZEBO/worlds/.
ls -al $SITL_GAZEBO/models/r1_rover/.
ls -al
set +x


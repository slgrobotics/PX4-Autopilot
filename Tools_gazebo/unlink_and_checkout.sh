#!/bin/bash

export SITL_GAZEBO=../Tools/simulation/gazebo-classic/sitl_gazebo-classic
export THIS_DIR=`pwd`

set -x
rm $SITL_GAZEBO/models/r1_rover/r1_rover.sdf.jinja
rm $SITL_GAZEBO/worlds/empty.world

cd $SITL_GAZEBO
git checkout models/r1_rover/r1_rover.sdf.jinja
git checkout worlds/empty.world
cd $THIS_DIR

ls -al $SITL_GAZEBO/worlds/.
ls -al $SITL_GAZEBO/models/r1_rover/.
ls -al
set +x


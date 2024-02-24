#!/bin/bash

export SITL_GAZEBO=../Tools/simulation/gazebo-classic/sitl_gazebo-classic

set -x
ls -al $SITL_GAZEBO/worlds/.
ls -al $SITL_GAZEBO/models/r1_rover/.
ls -al
set +x


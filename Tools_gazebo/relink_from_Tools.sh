#!/bin/bash

export SITL_GAZEBO=../Tools/simulation/gazebo-classic/sitl_gazebo-classic

echo "STOP: this script will replace files in THIS folder with files from $SITL_GAZEBO"

read -p "Do you really want to do this? [y/N]:" -n 1 -r

if [[ $REPLY =~ ^[Yy]$ ]]
then
    echo ""
    echo "Yes, proceeding"

    set -x
    ls -al $SITL_GAZEBO/models/r1_rover/r1_rover.sdf.jinja
    rm r1_rover.sdf.jinja
    ln $SITL_GAZEBO/models/r1_rover/r1_rover.sdf.jinja .
    
    rm empty.world
    ln $SITL_GAZEBO/worlds/empty.world .
    set +x
    ls -al

else
    echo ""
    echo "No, canceling"
fi

exit 0


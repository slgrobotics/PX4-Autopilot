#!/bin/bash

set -x

#make check_format
#make format

make px4_sitl gz_lawnmower_lawn
#make px4_sitl gz_lawnmower
#make px4_sitl gz_r1_rover
#make px4_sitl gz_rover_differential

#make emlid_navio2_arm64
#make emlid_navio2_arm64 upload

#make scumaker_pilotpi
#make scumaker_pilotpi_arm64
#make px4_raspberrypi
#make px4_raspberrypi_arm64

set +x

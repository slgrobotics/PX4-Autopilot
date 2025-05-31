#!/bin/bash

#
# a cleanup script to initiate AUTO SETUP of PX4 autopilot on Raspberry Pi
#

if [[ "$EUID" -ne 0 ]]; then
	echo
	echo " *********** Please run as sudo ************** "
	exit 1
fi

set -x

cd /home/pi/px4wrk/eeprom
rm parameters_50005

set +x


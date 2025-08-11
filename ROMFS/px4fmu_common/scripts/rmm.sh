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

rm -rf px4
rm -rf px4wrk

rm /fs/mtd_params
rm -rf /fs/microsd/*
rm -rf /fs/mtd_caldata

set +x


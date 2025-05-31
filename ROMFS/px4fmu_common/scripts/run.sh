#!/bin/bash

#
# a startup script to run PX4 autopilot on Raspberry Pi
#

if [[ "$EUID" -ne 0 ]]; then
	echo
	echo " *********** Please run as sudo ************** "
	exit 1
fi

echo "FYI: initially running at nice=`nice`"

set -x

#
# set GPIO 26 to 1 to enable GPS on /dev/ttyS0, pin 20 for debugging (yellow LED)
#
# on my RPi Hat a CD4066B chip is used to disable GPS RX/TX serial to have
#    it not interfere with UBoot. Setting GPIO26 enables the serial connection.
#
# see http://wiringpi.com/the-gpio-utility/
#

gpio -g mode 20 out
gpio -g mode 26 out
gpio -g write 26 1


export PATH=/home/pi/px4/bin:$PATH

echo "PATH=" $PATH

#
echo "IP: setting parameters..."
#

#export PX4_SIM_MODEL="aion_robotics_r1_rover"
#export PX4_ESTIMATOR="ekf2"

# Define Lawnmower for autostart (param SYS_AUTOSTART):
export PX4_MODEL_ID=50005

#
echo "IP: starting PX4..."
#

cd /home/pi/px4wrk

#
# first parameter - script folder, where without "-s ..." px4 will run init.d/rcS first
# -w defines folder where "eeprom". "log", "dataman" will be created
# the rcS will see vehicle code and find 50005 - so it calls etc/init.d-posix/50005_lawnMower1_config
# you could specify -s and skip rcS, but then you are on your own :-)
#

nice -n -20 /home/pi/px4/bin/px4 /home/pi/px4wrk/etc -s /home/pi/px4wrk/etc/init.d/rcS -w /home/pi/px4wrk

echo "OK: exited PX4, terminating"
#

set +x

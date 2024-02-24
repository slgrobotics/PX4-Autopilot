#!/bin/bash

#
# a script to create fresh px4wrk directory
#
# the px4 directory can be removed and will be recreated during the build
#     it remains a run/executable directory, while px4wrk is used as working dir
#

set -x

# create work directories:

mkdir px4wrk
mkdir px4wrk/log
mkdir px4wrk/eeprom
mkdir px4wrk/etc
mkdir px4wrk/etc/extras
mkdir px4wrk/etc/init.d
mkdir px4wrk/etc/init.d/airframes
#mkdir px4wrk/etc/init.d-posix
#mkdir px4wrk/etc/init.d-posix/airframes

# now populate work directories with neccessary files:

bash ~/update_wrk.sh

#!/bin/bash

#
# a script to deploy all maintenance scripts to home directory.
#
# Run this script in ~/px4/etc/scripts/ after "make emlid_navio2 upload" has finished:
#
#	cd ~/px4/etc/scripts/
#	bash deploy_scripts.sh
#

set -x

cp ~/px4/etc/scripts/*.sh ~/.
chmod +x ~/*.sh

#!/bin/bash

#
# a script to copy newly built essential files from px4 to px4wrk
#
# the px4 directory can be removed and will be recreated during the build
#     it remains a run/executable directory, while px4wrk is used as working dir
#
# the px4wrk directory can be removed and recreated using create_px4wrk.sh
#

set -x

# "extras" are json files that often change by the build:
cp --preserve ~/px4/etc/extras/* ~/px4wrk/etc/extras/.

# we only need rc.* files and rcS (very important startup) under init.d:
cp --preserve ~/px4/etc/init.d/rc.* ~/px4wrk/etc/init.d/.
cp --preserve ~/px4/etc/init.d/rcS ~/px4wrk/etc/init.d/.

# Modify auto-generated autostart files to run in px4wrk directory:
cat ~/px4/etc/init.d/rc.autostart | sed 's/ \/etc/ $\{R\}\/etc/' > ~/px4wrk/etc/init.d/rc.autostart
cat ~/px4/etc/init.d/rc.autostart.post | sed 's/ \/etc/ $\{R\}\/etc/' > ~/px4wrk/etc/init.d/rc.autostart.post

# we need the airframe here:
cp --preserve ~/px4/etc/init.d/airframes/50005_* ~/px4wrk/etc/init.d/airframes/.


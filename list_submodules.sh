#!/bin/bash

echo "List all submodules hashes"

#find .git -name HEAD -print|sed s/^.git.modules//

#find .git -name HEAD -print|grep -v "logs"|grep -v "origin"

#find .git -name HEAD -print|grep -v "logs"|grep -v "origin"| xargs cat


for FILE in $(find .git -name HEAD -print|grep -v "logs"|grep -v "origin"); do
	echo -n `cat $FILE` 
	echo -n "   "
	echo $FILE | cut -c14- | sed s/.HEAD$//
done

exit 0

#
# fixing git merge:
#
# git remote -v
# git remote add upstream https://github.com/PX4/PX4-Autopilot.git
# git remote -v
# git fetch upstream    (maybe  --single-branch --recursive -b main )
# git status
# git merge upstream/main


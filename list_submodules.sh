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


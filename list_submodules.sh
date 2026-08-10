#!/bin/bash

echo "List all submodules hashes"

set -x
git submodule status
set +x

#find .git -name HEAD -print|sed s/^.git.modules//

#find .git -name HEAD -print|grep -v "logs"|grep -v "origin"

#find .git -name HEAD -print|grep -v "logs"|grep -v "origin"| xargs cat

echo "================= New script: ======================================"
# Find only the true HEAD file at the base of each submodule metadata directory
find .git/modules -type f -name HEAD 2>/dev/null | while read -r FILE; do
    
    # Extract the base directory containing the valid HEAD file
    SUBMOD_GIT_DIR=$(dirname "$FILE")
    
    # Safeguard: Skip it if it's accidentally a logs path
    if [[ "$SUBMOD_GIT_DIR" == */logs* ]]; then
        continue
    fi
    
    # Use Git to resolve the definitive 40-character SHA-1 hash
    HASH=$(git --git-dir="$SUBMOD_GIT_DIR" rev-parse HEAD 2>/dev/null)
    
    # If the hash was successfully resolved, format and print it
    if [ -n "$HASH" ]; then
        CLEAN_PATH=${SUBMOD_GIT_DIR#.git/modules/}
        echo "${HASH}   ${CLEAN_PATH}"
    fi
done

echo "================= Old script: ======================================"
for FILE in $(find .git -name HEAD -print|grep -v "logs"|grep -v "origin"); do
	echo -n `cat $FILE` 
	echo -n "   "
	echo $FILE | cut -c14- | sed s/.HEAD$//
done

exit 0

=================================================================
# 1. Move into the submodule directory
cd src/drivers/gps/devices

# 2. Fetch the latest commits from the remote repository
git fetch

# 3. Checkout the specific commit hash
git checkout <good hash>

# 4. Move back to the main repository root
cd -

# 5. Stage the submodule update in the main repository
git add src/drivers/gps/devices

# 6. Commit the change
git commit -m "Update GPS devices submodule to hash 9b827163"
=================================================================


#
# fixing git merge:
#
# git remote -v
# git remote add upstream https://github.com/PX4/PX4-Autopilot.git
# git remote -v
# git fetch upstream    (maybe  --single-branch --recursive -b main )
# git status
# git merge upstream/main


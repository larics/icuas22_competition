#/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

wget -T 30 -O $(pwd)/custom_config/icuas2022_arena_latest.binvox.bt https://ferhr-my.sharepoint.com/:u:/g/personal/aivanovic_fer_hr/EVyKNebAQbJClA-jIXe7PYcBYaWyeZRe4v7gBY21heAr9g?e\=99ReSs\&download\=1

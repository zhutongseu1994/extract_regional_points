#########################################################################
# File Name: rosbagplay.sh
# Author: zll
# mail: zhnlion@126.com
# Created Time: 2020年09月22日 星期二 15时08分03秒
#########################################################################
#!/bin/bash

while :
do
rosbagplay_running=$(ps -ef | grep "rosbag play" | grep -v "grep")
if [$rosbagplay_running];then
    echo "$rosbagplay_running is  running!"
else
    rosbag play $1
fi

sleep 1
done

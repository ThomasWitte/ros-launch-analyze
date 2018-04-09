#!/bin/bash

# This script starts an automatic evaluation on the ros_tutorials project.
# start it using "source ./ros_tutorials.sh" to avoid errors due to missing
# roscd etc.
# The results are written to the file rospy.log in the current directory.

if [ "`type -t roscd`" != "function" ]; then
    echo "ROS paths are not set; start the script using \"source $0\""
    exit
fi

olddir=`pwd`

roscd rospy_tutorials
for f in `find 0* -name "*.launch"` ; do
    logfile="$olddir/rospy_$(basename $f .launch)"
    rosrun ros_launch_lint lint --print-dot --no-print-node-tree --no-print-topics -o "$logfile.dot" $f
    dot -Tps "$logfile.dot" > "$logfile.ps"

    logfile2="$olddir/rospy_reference_$(basename $f .launch)"
    rm "$logfile2.log" "$logfile2.ps"
    (timeout 5 roslaunch $f)&
    sleep 2
    for n in `rosnode list` ; do
        rosnode info $n >> $logfile2.log
    done
    sleep 3
    cat "$logfile2.log" | lua "$olddir/roslog2dot.lua" | dot -Tps > "$logfile2.ps"
done
cd -

#!/bin/bash

# This script starts an automatic evaluation on the ros_tutorials project.
# start it using "source ./eval_hector_quadrotor.sh" to avoid errors due to missing
# roscd etc.
# The results are written to the file rospy.log in the current directory.

if [ "`type -t roscd`" != "function" ]; then
    echo "ROS paths are not set; start the script using \"source $0\""
    exit
fi

olddir=`pwd`
launchfile="launch/indoor_slam_gazebo.launch"

roscd hector_quadrotor_demo

logfile="$olddir/hector_quadrotor_$(basename $launchfile .launch)"
rosrun ros_launch_lint lint --print-dot --no-print-node-tree --no-print-topics -o "$logfile.dot" $launchfile
dot -Tps "$logfile.dot" > "$logfile.ps"

logfile2="$olddir/hector_quadrotor_reference_$(basename $launchfile .launch)"
rm "$logfile2.log" "$logfile2.ps"
(timeout 25 roslaunch $launchfile)&
sleep 5
for n in `rosnode list` ; do
    rosnode info $n >> $logfile2.log
done
sleep 20
cat "$logfile2.log" | lua "$olddir/../tools/roslog2dot.lua" | dot -Tps > "$logfile2.ps"

cd -

#!/bin/bash

mkdir -p ../results

logfile="../results/topic_cpp"
rosrun ros_launch_lint lint --print-dot --no-print-node-tree --no-print-topics -o "$logfile.dot" topic.launch
rosrun ros_launch_lint lint --print-node-tree --print-topics -o "$logfile.txt" topic.launch
dot -Tps "$logfile.dot" > "$logfile.ps"


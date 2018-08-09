#!/bin/bash

mkdir -p ../results

logfile="../results/topic"
rosrun ros_launch_lint lint --compare -o "$logfile.txt" topic.launch

logfile="../results/remap"
rosrun ros_launch_lint lint --compare -o "$logfile.txt" remap.launch

logfile="../results/args"
rosrun ros_launch_lint lint --compare -o "$logfile.txt" args.launch


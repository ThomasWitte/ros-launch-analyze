#!/bin/bash

mkdir -p ../results

logfile="../results/topic_cpp"
rosrun ros_launch_lint lint --compare -o "$logfile.txt" topic.launch


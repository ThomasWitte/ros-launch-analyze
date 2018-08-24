#!/bin/bash

mkdir -p results

rosrun ros_launch_lint lint --compare -o "results/rosparam.txt" rosparam/rosparam.launch
rosrun ros_launch_lint lint --compare -o "results/topic.txt" topic_cpp/topic.launch
rosrun ros_launch_lint lint --compare -o "results/remap.txt" topic_cpp/remap.launch
rosrun ros_launch_lint lint --compare -o "results/args.txt" topic_cpp/args.launch
rosrun ros_launch_lint lint --compare -o "results/topic_py.txt" topic_py/topic_py.launch


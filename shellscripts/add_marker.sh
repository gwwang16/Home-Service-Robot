#! /bin/bash

x-terminal-emulator -e roslaunch homebot homebot_world.launch &
sleep 3
x-terminal-emulator -e roslaunch homebot amcl.launch &
sleep 3
x-terminal-emulator -e rosrun add_marker add_marker_beta


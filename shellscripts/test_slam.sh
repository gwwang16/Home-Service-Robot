#! /bin/bash

x-terminal-emulator -e roslaunch homebot homebot_world.launch &
sleep 3
x-terminal-emulator -e roslaunch homebot gmapping_demo.launch &
sleep 3
x-terminal-emulator -e roslaunch homebot view_navigation.launch &
sleep 3
x-terminal-emulator -e roslaunch homebot keyboard_teleop.launch

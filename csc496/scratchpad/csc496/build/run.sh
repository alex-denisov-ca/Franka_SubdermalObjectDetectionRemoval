#!/bin/bash


# Ensure values are space-separated
origin_file="/root/git/scratchpad/csc496/build/origin.txt"
rough_capture_file="/root/git/scratchpad/csc496/build/rough_capture_location.txt"
capture_file="/root/git/scratchpad/csc496/build/capture_location.txt"

# touch $origin_file
# touch $rough_capture_file
# touch $capture_file

echo "##### PART 1: SET ORIGIN #####"
./run_set_origin $FRANKA_IP

read -r o_x o_y o_z < "$origin_file"
echo "##### PART 2: ROUGH CAPTURE #####"
./run_autoscan_rough_capture $FRANKA_IP "$o_x" "$o_y" "$o_z"

read -r r_x r_y r_z < "$rough_capture_file"
echo "##### PART 3: KEYBOARD-INPUT CAPTURE #####"
./run_keyboard_input_capture $FRANKA_IP "$r_x" "$r_y" "$r_z"

read -r x y z < "$capture_file"
echo "##### PART 4: OBJECT RETRIEVAL#####"
./run_grasp_given_xyz $FRANKA_IP "$x" "$y" "$o_z"
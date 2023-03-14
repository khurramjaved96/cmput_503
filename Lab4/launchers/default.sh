#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#roscore &
#sleep 5
#dt-exec rosrun my_package my_publisher_node.py
#dt-exec rosrun my_package my_subscriber_node.py
dt-exec roslaunch lane_follow lane_follow_node.launch veh:=csc22940
dt-exec roslaunch led_emitter led_emitter_node.launch veh:=csc22940
dt-exec roslaunch duckiebot_detection duckiebot_detection_node.launch veh:=csc22940
# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join

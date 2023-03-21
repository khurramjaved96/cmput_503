# Don’t Crash! Tailing Behaviour
## Introduction
In this exercise, we managed to drive the duckiebot autonomously in a way that can follow a leader robot at a safe distance while it is in its sight, and also follow duckietown rules including stopping at intersections and changing LED lights when turning.

In summary,  for stopping at intersections, we used the apriltag detections as landmarks to indicate if the robot is at an intersection and should stop or not.
For maintaining a safe distance we used the duckiebot_distance_node, and for changing the LEDs, we used a service and changed the LED based on the duckiebot current state.
We will provide more details for each utility of tailing behavior in the next sections.


Deliverable: Video
We recorded two videos. One shows the tailing behavior, and the other shows stopping at an intersection with an apriltag before turning right, and then tailing the leader for turning left. 

The tailing and lane following demo is available at: https://youtu.be/MJdsZcV8NqE whereas the right and left turn demo is available at: https://www.youtube.com/shorts/nBxF645L9DM

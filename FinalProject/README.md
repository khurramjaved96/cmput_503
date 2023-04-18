## Introduction
The main goal of this lab project is to perform autonomous driving based on various conditions of the road (walking pedestrians, obstacles in the way, etc) and different goals of the driver (which turns to make, which parking stall to park in).

Overall, working with the DuckieBots is quite slow. The bots themselves have many Docker containers running which greatly inhibits the bot’s performance. We found that many of these containers did not serve a particularly useful purpose for us. Because of this, we shut down many containers that were not needed. For example, we shut off the container running Portainer. Then, instead of using the DuckieTown Shell (dts) to build our programs, we simply used ssh to access the bot and developed our programs directly in a docker container using catkin.

Furthermore, due to the significant overhead imposed by Python, we wrote this project in C++, which significantly improved the performance of the program running on the bot. Furthermore, C++ provides more efficient and lower-level tools for controlling the Duckiebot compared to Python. We need to use libraries such as ros, cvbridge, and apriltag detection, but these are all available in C++.

Overall, the project code consists of three directories:

1. control_command_executer: This part defines a LaneControl class, used to control the robot based on its state. This class directly controls the bot’s wheels. The module is very lightweight and runs at 30Hz. When no control command is currently available, this class will stop the bot. This functionality ensures that the bot does not run blindly if control commands are not available, or other nodes have been shut down or are experiencing high latency.

2. duckietown_msgs: This part contains details about the type of messages used in the project. It is included and used in the main part of the project, and the source code comes from dt-ros-commons [1].

3. lane_control: This is the main node which determines the control commands to execute in different stages of the lab. There is another LaneControl class which implements two important functionalities:

4. Image Callback Method: In summary, this method takes in the image message, and decides which state the bot is in, then sets the control command values based on the state. The details of each state and its corresponding commands will be provided in the next sections.

5. State Change Method: This method sets the current state of the robot and keeps track of the time when a state change has occurred. We then can determine which control commands to forward to the control_command_executer package based on the state of the Duckiebot. The control commands created here are forwarded to the LaneControl node in the control_command_executer package to be actually executed.

Note: To run the nodes, it's important to pass the correct image transport. For example, the lane control node can be run as: 
rosrun lane_control lane_control _image_transport:=compressed

# rcll_refbox_comm_example
This is an example to connect to the refbox of the robocup logistics league. The communication to the refbox is implemented in the rcll_refbox_comm_node.
The other packages are requirements to be able to communicate. This was only tested on Ubuntu 20.04 with ros-noetic

# Prerequisits
Protobuf and openssl need to be installed on your system

# Installation
1. Add https://git.ist.tugraz.at/ais/utils to your `catkin_ws/src`. If there are problems with the build simply remove all folders except for `tug-protobuf` as you will only need that util.  
`sudo apt-get update && sudo apt-get -y install protobuf-compiler protobuf-c-compiler ros-noetic-roslint  python3-yaml  libprotoc-dev`

2. Copy the packages to your catkin_ws/src folder. After running catkin_make everything should build.

# Next steps

Several suggestions for next steps:
 1. Implement awareness of game state (e.g. is the game paused the robot should not move or do tasks)
 2. Depending on the game state some protobuf messages were not registered with the message register. In this case you will see an error in the console, look up the comp_id and msg_type in the error message and register the corresponding message.
 3. Implement sending prepare machine instruction (e.g. send a prepare machine instruction such that the BS dispenses a product to the output: look at refbox_msgs/msg/MachineInstruction.proto)
 4. Implement a way to reset machines if they go into broken state

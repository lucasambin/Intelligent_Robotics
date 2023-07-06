# ASSIGNMENT 1
Please read the report if you have any doubt.
This code implements a routine that lets Tiago navigate inside the environment. Tiago has to navigate from point Starting Pose to Pose_B.
We also develop the server so if you want to move from a position to another you don't have to kill the server, that remain alive listening for request from an action client.
If we are in the Starting Pose we implement the bonus part, otherwise we simply use the move_base stack.
When the robot reach the Pose_B, start to detect the cylindrical obstacles in the laser scanner range [-110°, 110°] and send the result of the detection to the action client.

The Pose_B has this structure
- Position: [x, y, z]
- Orientation: [x, y, z, w]

If you type a wrong Pose_B, the client stops.
The package name that contain all the files is called "assignment_1_group_27".
We put this package in the src (tiago_public_ws/src/) folder of our workspace (tiago_public_ws) and we build it.
We compile the package with the command: $> catkin build assignment_1_group_27

Then we follow the guide step-by-step:
- Start the simulation (cmd console 1): $> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full
- Navigation stack (cmd console 2): 	$> roslaunch tiago_iaslab_simulation navigation.launch
- Start the action server (cmd console 3): 	$> rosrun assignment_1_group_27 server_node
- Start the action client (cmd console 4): 	$> rosrun assignment_1_group_27 client_node 10.95 0.15 0 0 0 -0.26 1

The other pose tested in the video:
$> rosrun assignment_1_group_27 client_node 11.81 1.03 0 0 0 -0.16 1
$> rosrun assignment_1_group_27 client_node 11.21 -1.03 0 0 0 -0.35 1
$> rosrun assignment_1_group_27 client_node 11.21 -1.03 0 0 0 -0.16 1
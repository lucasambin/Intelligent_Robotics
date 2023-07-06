# ASSIGNMENT 2
Please read the report if you have any doubt.
This code implements a routine that let's Tiago navigate inside the environment and 
perform a routine, exploiting the MoveIt! library, that allow to pick and place each object
(according to the received sequence) from the pick-up table to its final destination in accordance with its colour (i.e., the red cube on the red cylinder table)

Tiago has to navigate from point Starting Pose to a pick a position previoisly defined, according with the recived sequence.
The server is the same of the previous assignment, to which the laser detection for cylindrical object has been removed.
As in the previous assignment the server isn't kill, that remain alive listening for request from an action client.
If we are in the Starting Pose we implement the bonus part of the previous assignment, otherwise we simply use the move_base stack.
The node_a is the action client in which are defined the global position and the head configuration for each objects (red, green, blue).
When the robot reach the desired position, the node_a send a request to the action server developed in the node_b, which
start to detect the markers placed on the top of these objects, with a specific head configuration for each position (red, green, blue).
Then the node_b return to node_a the marker detected and the node_a send all the information to the server developed in node_c, which
is responsible for the exploitation of pick&place routine.
Then, when the object is picked the node_a tells the server_node to reach the corresponding detection point for cylindrical table, detecte the place table and navigate in front of them and when it is reached, we place the object.
We repeat this process until the sequence of object to be pick&place is over.

## EXTRA POINT
We also implement the required extra point, using the server_node of the previous assignment, detecting the 3 cylinder table pose (We assume that are in fixed position).
Check the report for more information.


Each Pose has this structure, (we calculate the orientation using roll pitch and yaw)
- Position: [x, y, z]
- Orientation: [x, y, z, w]

Each Head configuration has this structure
- head_1_joint -> left/right[-1,24, 1,24]
- head_2_joint -> [-0,98, 0,72] for down/up
which are value in a specific range


The package name that contain all the files is called "assignment_2_group_27".
We put this package in the src (tiago_public_ws/src/) folder of our workspace (tiago_public_ws) and we build it.
We compile the package with the command: $> catkin build assignment_2_group_27

Then we follow the guide step-by-step:
- Start the simulation and MoveIt (cmd console 1):	$> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
- AprilTag (cmd console 2): 						$> roslaunch tiago_iaslab_simulation apriltag.launch
- Navigation stack (cmd console 3):					$> roslaunch tiago_iaslab_simulation navigation.launch
- Human Service node (cmd console 4): 				$> rosrun tiago_iaslab_simulation human_node

- Start the action server, server_node (cmd console 5): $> rosrun assignment_2_group_27 server_node
- Start the action server, node_b (cmd console 6): 	$> rosrun assignment_2_group_27 node_b
- Start the action server, node_c (cmd console 7): 	$> rosrun assignment_2_group_27 node_c
- Start the action client, node_a (cmd console 8): 	$> rosrun assignment_2_group_27 node_a

In case you want to change or upgrade this nodes, or if there is some error you can change them very quickly just looking to the node_a function -> "updateVariable()"
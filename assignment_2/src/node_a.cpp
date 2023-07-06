/**
 * Group 27
 * Luca Sambin
 * Andrea Fonsato
 * Zhaku Bejaj
*/

#include <ros/ros.h>
#include <tiago_iaslab_simulation/Objs.h> // Both request & response information
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <assignment_2_group_27/TiagoAction.h>
#include <assignment_2_group_27/DetectionAction.h>
#include <assignment_2_group_27/PickPlaceAction.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>

// Function that create a position [x, y, z] (point)
geometry_msgs::Point make_point(const float& x, const float& y, const float& z)
{
    geometry_msgs::Point p;
    p.x = x; 
    p.y = y; 
    p.z = z;
    return p;
}

// Function that assign the global position value and the correspondent Tiago head parameters
void updateVariable(geometry_msgs::Pose& blue_hexagon, float& blue_head_1_joint, float& blue_head_2_joint,
                    geometry_msgs::Pose& green_triangle, float& green_head_1_joint, float& green_head_2_joint,
                    geometry_msgs::Pose& red_cube, float& red_head_1_joint, float& red_head_2_joint, geometry_msgs::Pose& cylinder_detection_point,
                    geometry_msgs::Pose& waypoint)
{
    // Blue cube position, Blue cylindrical table position and corresponding head position
    blue_hexagon.position = make_point(8.27, -1.94, 0.0);
    blue_hexagon.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
    blue_head_1_joint = -0.55; // Turn head to right
    blue_head_2_joint = -0.70; // Turn head down

    // Green cube position, Green cylindrical table position and corresponding head position
    green_triangle.position = make_point(7.45, -4.01, 0.0);
    green_triangle.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI/2.0);
    green_head_1_joint = -0.40; // Turn head to right
    green_head_2_joint = -0.70; // Turn head down

    // Red cube position, Red cylindrical table position and corresponding head position
    red_cube.position = make_point(7.5, -1.965, 0.0);
    red_cube.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
    red_head_1_joint = 0.45; // Keep head in center
    red_head_2_joint = -0.75; // Turn head down

    // Cylinder detection point position
    cylinder_detection_point.position = make_point(10.685, -1.95, 0);
    cylinder_detection_point.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI/2.0);

    // Waypoint to avoid first obstacle after the narrow corridor
    waypoint.position = make_point(8.5, 0.2, 0);
    waypoint.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -M_PI/2.0);
}

// Function that check if the received ID is of a blue hexagon (1)
bool isBlue(const int& aID)
{
    if (aID == 1)
    {
        return true;
    }
    else
    {
        return false;
    }  
}

// Function that check if the received ID is of a green triangle (2)
bool isGreen(const int& aID)
{
    if (aID == 2)
    {
        return true;
    }
    else
    {
        return false;
    }    
}

// Function that check if the received ID is of a red cube (3)
bool isRed(const int& aID)
{
    if (aID == 3)
    {
        return true;
    }
    else
    {
        return false;
    }    
}

// Function that is responsible for the pick&place of the object (node_c)
bool pick_place_object(const assignment_2_group_27::PickPlaceGoal& aGoal, assignment_2_group_27::PickPlaceGoal& aGoalPlace)
{
    // Create the action client. We also tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<assignment_2_group_27::PickPlaceAction> ac_pickplace("node_c", true);
    ROS_INFO("Waiting for node_c action server to start");
    // Wait for the action server to come up
    ac_pickplace.waitForServer(); // Will wait for infinite time
    ROS_INFO("node_c action server started, sending goal");

    // Send a goal to the action server
    ac_pickplace.sendGoal(aGoal);
    
    // Wait for the action to return
    ac_pickplace.waitForResult();
    
    // If the goal finished before the time out the goal status is reported
    if (ac_pickplace.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        actionlib::SimpleClientGoalState state = ac_pickplace.getState();
        //ROS_INFO("node_c action server finish: %s",state.toString().c_str());
        // ONLY IF PICK IS PERFORMED
        if (aGoal.pick == true && aGoal.place == false)
        {
            // Store the offset on z axis (base_footprint frane) wrt to the table surface.
            // This value is used to make a more precise place
            assignment_2_group_27::PickPlaceResultConstPtr result_pick = ac_pickplace.getResult();
            aGoalPlace.z_object_place_contact_point = result_pick->z_object_pick_contact_point;
            ROS_INFO("z_object_pick_contact_point: %f", aGoalPlace.z_object_place_contact_point);
        }
        return true;
    }
    // Otherwise the user is notified that the goal did not finish in the allotted time.
    else
    {
        ROS_INFO("node_c action server did not finish before the time out");
        return false;
    }
}

// Called once when the goal completes -> At the end
void markerDoneCb(const actionlib::SimpleClientGoalState& state,
            const assignment_2_group_27::DetectionResultConstPtr& result)
{
    // The desired marker ID was detected
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        //ROS_INFO("node_b finish: %s",state.toString().c_str());
        ROS_INFO("Number of Marker detected: %i", (int) result->obs_ID.size() + 1);
        // Desired marker detected
        ROS_INFO("Marker ID: %i, Size: %f, Pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
                result->marker_ID, result->marker_size,
                result->marker_pose.position.x, result->marker_pose.position.y,
                result->marker_pose.position.z, result->marker_pose.orientation.x, 
                result->marker_pose.orientation.y, result->marker_pose.orientation.z, 
                result->marker_pose.orientation.w);

        // Other marker detected
        for(int i=0; i < result->obs_ID.size(); i++)
        {
            // Print for each obstacle, its position
            ROS_INFO("Marker ID: %i, Size: %f, Pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
                    result->obs_ID.at(i), result->obs_size.at(i),
                    result->obs_pose.at(i).position.x, result->obs_pose.at(i).position.y,
                    result->obs_pose.at(i).position.z, result->obs_pose.at(i).orientation.x, 
                    result->obs_pose.at(i).orientation.y, result->obs_pose.at(i).orientation.z, 
                    result->obs_pose.at(i).orientation.w);
        }
    }   
}
void markerActiveCb(){}
void markerFeedbackCb(const assignment_2_group_27::DetectionFeedbackConstPtr& feedback){}

// Function that is responsible for the detection of the object (node_b)
assignment_2_group_27::DetectionResultConstPtr marker_detection(const assignment_2_group_27::DetectionGoal& aGoal)
{
    // Create the action client. We also tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<assignment_2_group_27::DetectionAction> ac_detection("node_b", true);
    ROS_INFO("Waiting for node_b server to start");
    // Wait for the action server to come up
    ac_detection.waitForServer(); // Will wait for infinite time
    ROS_INFO("node_b server started, sending goal");

    // Send a goal to the node_b action server
    //ac_detection.sendGoal(aGoal);
    ac_detection.sendGoal(aGoal, &markerDoneCb, &markerActiveCb, &markerFeedbackCb);
    
    // Wait for the action to return
    bool finished_before_timeout = ac_detection.waitForResult(ros::Duration(150.0));
    
    // If the goal finished before the time out the goal status is reported
    if (finished_before_timeout)
    {
        return ac_detection.getResult();
    }
    // Otherwise the user is notified that the goal did not finish in the allotted time.
    else
    {
        ROS_INFO("node_b action server did not finish before the time out");
        return NULL;
    }
}

// Function that check if the robot is in the Starting Position
bool checkRobotPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    float ths = 0.1;
    if (fabs(pose_msg->pose.pose.position.x) < ths &&
        fabs(pose_msg->pose.pose.position.y) < ths)
    {
        return true;
    }
    return false;
}

// Function that is responsible for the motion of Tiago (server_node)
bool move_tiago(const assignment_2_group_27::TiagoGoal& aGoal)
{
    // Create the action client. We also tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<assignment_2_group_27::TiagoAction> ac_move("server_node", true);
    ROS_INFO("Waiting for server_node action server to start");
    // Wait for the action server to come up
    ac_move.waitForServer(); // Will wait for infinite time
    ROS_INFO("server_node action server started, sending goal");

    // Send a goal to the action server
    ac_move.sendGoal(aGoal);
    
    // Wait for the action to return
    bool finished_before_timeout = ac_move.waitForResult(ros::Duration(300.0));
    
    // If the goal finished before the time out the goal status is reported
    if (finished_before_timeout)
    {
        /*
        actionlib::SimpleClientGoalState state = ac_move.getState();
        ROS_INFO("server_node action server finish: %s",state.toString().c_str());
        */
        return true;
    }
    // Otherwise the user is notified that the goal did not finish in the allotted time.
    else
    {
        ROS_INFO("server_node action server did not finish before the time out");
        return false;
    }
}

// EXTRA POINT
// Function that return the place position in front of the correspondent cylinder
geometry_msgs::Point find_cylinder_position(const assignment_2_group_27::TiagoResultConstPtr& result, const int& aID)
{
    geometry_msgs::Point tmp;
    if (result->obstacle.size() != 3)
    {
        ROS_INFO("Miss some place cylinder!");
        tmp.z = -1.0;
        return tmp;
    }    
    // Check if the i-th ID correspond to a blue hexagon (1)
    if (isBlue(aID))
    {
        ROS_INFO("blue cylinder! Position: [x: %f, y: %f]", result->obstacle.at(0).x, result->obstacle.at(0).y);
        // It's the first cylinder detected
        tmp = result->obstacle.at(0);
    }
    // Check if the i-th ID correspond to a green triangle (2)
    else if (isGreen(aID))
    {
        ROS_INFO("green triangle! Position: [x: %f, y: %f]", result->obstacle.at(1).x, result->obstacle.at(1).y);
        // It's the second cylinder detected
        tmp = result->obstacle.at(1);
    }
    // Check if the i-th ID correspond to a red cube (3)
    else if (isRed(aID))
    {
        ROS_INFO("red cube! Position: [x: %f, y: %f]", result->obstacle.at(2).x, result->obstacle.at(2).y);
        // It's the third cylinder detected
        tmp = result->obstacle.at(2);
    }
    return tmp;
}

// EXTRA POINT
// Function that is responsible for the motion of Tiago (server_node) and detection of cylinder
geometry_msgs::Point move_tiago_detection(const assignment_2_group_27::TiagoGoal& aGoal, const int& aID)
{
    geometry_msgs::Point tmp;
    // Create the action client. We also tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<assignment_2_group_27::TiagoAction> ac_move_det("server_node", true);
    ROS_INFO("Waiting for server_node action server to start");
    // Wait for the action server to come up
    ac_move_det.waitForServer(); // Will wait for infinite time
    ROS_INFO("server_node action server started, sending goal");

    // Send a goal to the action server
    ac_move_det.sendGoal(aGoal);
    
    // Wait for the action to return
    bool finished_before_timeout = ac_move_det.waitForResult(ros::Duration(300.0));
    
    // If the goal finished before the time out the goal status is reported
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_move_det.getState();
        ROS_INFO("server_node action server finish: %s",state.toString().c_str());
        assignment_2_group_27::TiagoResultConstPtr result = ac_move_det.getResult();
        tmp = find_cylinder_position(ac_move_det.getResult(), aID);
        return tmp;
    }
    // Otherwise the user is notified that the goal did not finish in the allotted time.
    else
    {
        ROS_INFO("server_node action server did not finish before the time out");
        tmp.z = -1.0;
        return tmp;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    // Creates a client for the "human_objects_srv" service
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    tiago_iaslab_simulation::Objs srv;
    // If true human_node send the object's ID
    srv.request.ready = true;
    // Set true only for extra point
    srv.request.all_objs = true;
    // Save the sequence of ID to pick up
    std::vector<int> ids;
    
    // Call the service and wait the response, because service are blocking
    if (client.call(srv))
    {
        ROS_INFO("ID size: %i", (int) srv.response.ids.size());
        // Print on terminal our response
        for (int i=0; i < srv.response.ids.size(); i++)
        {
            ids.push_back(srv.response.ids.at(i));
            ROS_INFO("ID %i: %i", i+1, (int) srv.response.ids.at(i));
        }
    }
    else
    {
        ROS_ERROR("Failed to call service");
        // Kill our client_node
        return 1;
    }

    // Define the five fixed position for pick and place objects and the correspondent Tiago head parameter
    geometry_msgs::Pose blue_hexagon;
    float blue_head_1_joint;
    float blue_head_2_joint;
    geometry_msgs::Pose green_triangle;    
    float green_head_1_joint;
    float green_head_2_joint;    
    geometry_msgs::Pose red_cube;
    float red_head_1_joint;
    float red_head_2_joint;
    geometry_msgs::Pose waypoint;
    geometry_msgs::Pose cylinder_detection_point;

    // Update the global position value and the correspondent Tiago head and arm parameters
    updateVariable(blue_hexagon, blue_head_1_joint, blue_head_2_joint,
                   green_triangle, green_head_1_joint, green_head_2_joint,
                   red_cube, red_head_1_joint, red_head_2_joint, cylinder_detection_point,
                   waypoint);

    // Define the goal pose
    assignment_2_group_27::TiagoGoal goal_object;
    assignment_2_group_27::TiagoGoal goal_cylinder;
    assignment_2_group_27::TiagoGoal goal_waypoint;    
    
    // Define the detection goal
    assignment_2_group_27::DetectionGoal goal_ID;
    
    for (int i=0; i < ids.size(); i++)
    {
        // Use server_node to navigate in front of the object   
        goal_object.detection = false;

        // Check if the i-th ID correspond to a blue hexagon (1)
        if (isBlue(ids.at(i)))
        {
            ROS_INFO("ID: %i is a blue hexagon!", ids.at(i));
            goal_object.pose_B = blue_hexagon;
            goal_ID.head_1_joint = blue_head_1_joint;
            goal_ID.head_2_joint = blue_head_2_joint;
        }
        // Check if the i-th ID correspond to a green triangle (2)
        else if (isGreen(ids.at(i)))
        {
            ROS_INFO("ID: %i is a green triangle!", ids.at(i));
            goal_object.pose_B = green_triangle;
            goal_ID.head_1_joint = green_head_1_joint;
            goal_ID.head_2_joint = green_head_2_joint;
        }
        // Check if the i-th ID correspond to a red cube (3)
        else if (isRed(ids.at(i)))
        {
            ROS_INFO("ID: %i is a red cube!", ids.at(i));
            goal_object.pose_B = red_cube;
            goal_ID.head_1_joint = red_head_1_joint;
            goal_ID.head_2_joint = red_head_2_joint;
        }
        // Otherwise error and stop the client
        else 
        {
            ROS_INFO("Fail! ID: %i is not a red cube, a green triangle or a blue hexagon", ids.at(i));
            return 1;
        }
        
        // Only when Tiago is in the Starting position, use a way point to avoid Tiago to get stuck 
        // in the first cylindric obstacle right after the narrow corridor

        // Pointer to a robot pose message
        geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg = NULL;
        // We will wait until we receive some pointer from the /robot_pose topic
        while (pose_msg == NULL)
        { 
            pose_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", nh, ros::Duration(5.0));
        }

        // If the robot is in the origin position then navigate through the narrow corridor
        // until the way point is reached, then move to the object ID location
        if (checkRobotPosition(pose_msg))
        {
            goal_waypoint.pose_B = waypoint;
            ROS_INFO("ID: %i, Start waypoint move!", ids.at(i));
            if (! move_tiago(goal_waypoint))
            {
                ROS_INFO("ID: %i, Fail waypoint move!", ids.at(i));
                return 1;
            }
            ROS_INFO("ID: %i, End waypoint move!", ids.at(i));
        }

        // Move from current position to i-th object position
        ROS_INFO("ID: %i, Start pick move!", ids.at(i));
        if (! move_tiago(goal_object))
        {
            // Failed to reach the i-th object position
            ROS_INFO("ID: %i, Fail first move!", ids.at(i));
            return 1;
        }
        ROS_INFO("ID: %i, End pick move!", ids.at(i));
        
        // Detection of the obstacle -> node_b
        // Moving the torso and the head of Tiago and perform the detection
        ROS_INFO("ID: %i, Start detection!", ids.at(i));
        goal_ID.marker_ID = ids.at(i);
        
        // Save the result of the detection inside this pointer
        assignment_2_group_27::DetectionResultConstPtr detection_ptr = marker_detection(goal_ID);
        if (detection_ptr == NULL)
        {
            ROS_INFO("ID: %i, Detection failed!", ids.at(i));
            return 1;
        }
        ROS_INFO("ID: %i, End detection!", ids.at(i));
        
        // Only pick when the objcet is detected
        // Use the result of the detection to pick the i-th object
        assignment_2_group_27::PickPlaceGoal goal_pick;
        goal_pick.pick = true;
        goal_pick.place = false;
        goal_pick.marker_ID = detection_ptr->marker_ID;
        goal_pick.marker_size = detection_ptr->marker_size;
        goal_pick.marker_pose = detection_ptr->marker_pose;
        goal_pick.obs_ID = detection_ptr->obs_ID;
        goal_pick.obs_size = detection_ptr->obs_size;
        goal_pick.obs_pose = detection_ptr->obs_pose;
        assignment_2_group_27::PickPlaceGoal goal_place;
        goal_place.pick = false;
        goal_place.place = true;
        goal_place.marker_ID = detection_ptr->marker_ID;           
        goal_pick.marker_size = detection_ptr->marker_size;
        goal_pick.marker_pose = detection_ptr->marker_pose;


        ROS_INFO("ID: %i, Start Pick!", ids.at(i));
        if (! pick_place_object(goal_pick, goal_place))
        {
            // Pick is failed
            ROS_INFO("ID: %i, Pick is failed!", ids.at(i));
            return 1;
        }
        ROS_INFO("ID: %i, Pick successfully!", ids.at(i));

        // EXTRA POINT
        // Move from i-th object position to detection point position
        ROS_INFO("ID: %i, Start EXTRA point move!", ids.at(i));
        goal_cylinder.pose_B = cylinder_detection_point;
        goal_cylinder.detection = true;        
        geometry_msgs::Point cylinder_place_pose = move_tiago_detection(goal_cylinder, ids.at(i));
        
        if (cylinder_place_pose.z == -1.0)
        {
            // Detection of cylinder is failed
            ROS_INFO("ID: %i, Detection of cylinder is failed!", ids.at(i));
            return 1;
        }
        ROS_INFO("ID: %i, End EXTRA point move!", ids.at(i));

        goal_cylinder.detection = false;
        // Set the target position of the robot in front of the cylindrical table
        // cylinder_place_pose -> x, y, radius
        // Apply a offset to place position (radius + 0.5m) on x and - 10cm on y
        float safe_ths = 0.45;
        goal_cylinder.pose_B.position.x = cylinder_place_pose.x - 0.15;
        goal_cylinder.pose_B.position.y = cylinder_place_pose.y - (cylinder_place_pose.z + safe_ths);
        goal_cylinder.pose_B.position.z = 0.0;
        
        // Move from i-th object position to its cylindrical table position
        ROS_INFO("ID: %i, Start place move!", ids.at(i));
        if (! move_tiago(goal_cylinder))
        {
            // Failed to reach the i-th cylindrical table position
            ROS_INFO("ID: %i, Fail place move!", ids.at(i));
            return 1;
        }
        ROS_INFO("ID: %i, End place move!", ids.at(i));
        
        // Place the object
        ROS_INFO("ID: %i, Start Place!", ids.at(i));
        goal_place.cylinder_place_point = cylinder_place_pose;
        // Because of server structure, place_holder does nothing
        assignment_2_group_27::PickPlaceGoal place_holder;
        if(! pick_place_object(goal_place, place_holder))
        {
            // Place is failed
            ROS_INFO("ID: %i, Place is failed!", ids.at(i));
            return 1;
        }
        ROS_INFO("ID: %i, Place successfully!", ids.at(i));
    }

    // All the three object are picked and placed
    ROS_INFO("All objects are picked anb placed");
    return 0;
}
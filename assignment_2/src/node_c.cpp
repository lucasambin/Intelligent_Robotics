/**
 * Group 27
 * Luca Sambin
 * Andrea Fonsato
 * Zhaku Bejaj
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <assignment_2_group_27/PickPlaceAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class PPAction
{
    private:
        // Struct used to store the angle value necessary to achieve a better result
        struct RPY_angle
        {
            double roll;
            double pitch;
            double yaw;
        };

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_2_group_27::PickPlaceAction> as_;
        std::string action_name_;
        assignment_2_group_27::PickPlaceResult result_;
        assignment_2_group_27::PickPlaceFeedback feedback_;
        
        // Offset along X
        float x_gripper_offset = 0.2;

        // Table object in map coordinate
        const geometry_msgs::Point table_position = PPAction::make_point(7.826, -2.983, (0.775)/2.0);
        const geometry_msgs::Point table_surface_position = PPAction::make_point(0.913, 0.913, 0.795);
        const geometry_msgs::Quaternion table_orientation = PPAction::make_quaternion_from_xyz({0.0, 0.0, 0.0});
        
        // Cylinder height
        const float cylinder_height = 0.69;
        
        // Select group of joints
        std::string PLANNING_GROUP; // arm, arm_torso, gripper

        // Default value, which are updated wrt detection message (base_footprint)
        // All object
        const float x_angle = M_PI/2.0; // 90° gripper orientato per prendere i vari oggetti
        const float y_green_angle = M_PI/2.0; // 90° TOP approach
        const float y_angle =  M_PI/8.0; // 
        // Only for green object
        const float green_z_angle = -M_PI/7.0;

        // Defined wrt to base_footprint
        RPY_angle blue_pick_orientation{x_angle, y_angle, 0.0};
        RPY_angle green_pick_orientation{x_angle, y_green_angle, green_z_angle};
        RPY_angle red_pick_orientation{x_angle, y_green_angle, 0.0};
        RPY_angle place_front_orientation{x_angle, y_angle, 0.0};
        RPY_angle place_top_orientation{x_angle, y_green_angle, 0.0};

        
        // Blue intermediate_pose
        const geometry_msgs::Point left_intermediate_point = PPAction::make_point(0.3, 0.6, 0.98);
        const geometry_msgs::Quaternion left_intermediate_orientation = PPAction::make_quaternion_from_xyz({x_angle, 0, M_PI/2.0});
        // Green and Red intermediate_pose
        const geometry_msgs::Point right_intermediate_point = PPAction::make_point(0.3, -0.6, 0.98);        
        const geometry_msgs::Quaternion right_intermediate_orientation = PPAction::make_quaternion_from_xyz({x_angle, 0, -M_PI/2.0});

        
        ros::Rate r = ros::Rate(1);
        bool initialized = false;
        // Define the target frame (map) and source frame (base_footprint)
        const std::string target_frame = "base_footprint";
        const std::string source_frame = "map";
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::planning_interface::MoveGroupInterface group_arm;


        // Function that transform the poses obtained by "updateArrayTags", from camera frame to base_link frame before sending them to node_a
        geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, const tf::StampedTransform& transform) 
        {
            tf::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
            v = transform * v;
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            q = transform * q;

            geometry_msgs::Pose tmp_pose;
            tmp_pose.position.x = v.x();
            tmp_pose.position.y = v.y();
            tmp_pose.position.z = v.z();
            tmp_pose.orientation.x = q.x();
            tmp_pose.orientation.y = q.y();
            tmp_pose.orientation.z = q.z();
            tmp_pose.orientation.w = q.w();
            return tmp_pose;
        }

        // Function that create a position [x, y, z] (point)
        geometry_msgs::Point make_point(const float& x, const float& y, const float& z)
        {
            geometry_msgs::Point p;
            p.x = x; 
            p.y = y; 
            p.z = z;
            return p;
        }

        // Function that create an orientation [x, y, z, w] (quaternion)
        //geometry_msgs::Quaternion make_quaternion_from_xyz(const float& x, const float& y, const float& z)
        geometry_msgs::Quaternion make_quaternion_from_xyz(const RPY_angle& value)
        {
            return tf::createQuaternionMsgFromRollPitchYaw(value.roll, value.pitch, value.yaw);
        }

        // Function used to fix pose of table and cylindrical place table wrt base_footprint
        geometry_msgs::Pose fixPose(const geometry_msgs::Pose& pose)
        {
            tf::TransformListener listener;
            tf::StampedTransform transform;
            
            try
            {
                listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
                listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
            }
            return transformPose(pose, transform);
        }

        // RPY -> ROLL(X), PITCH(Y), YAW(Z)
        // Function that compute offset of parker wrt to base_footprint
        RPY_angle computeOffset(const geometry_msgs::Quaternion& aOrientation)
        {
            tf::Quaternion aQ(
                        aOrientation.x,
                        aOrientation.y,
                        aOrientation.z,
                        aOrientation.w);
            tf::Matrix3x3 aMatrix(aQ);
            double roll, pitch, yaw;
            aMatrix.getRPY(roll, pitch, yaw);
            RPY_angle tmp{roll, pitch, yaw};
            return tmp;
        }

        // Function necessary to give correct value to robot (approximation to 0.**m so 1cm)
        void roundPoint(geometry_msgs::Point& aPoint)
        {
            aPoint.x = ceil(aPoint.x * 1000.0) / 1000.0;
            aPoint.y = ceil(aPoint.y * 1000.0) / 1000.0;
            aPoint.z = ceil(aPoint.z * 1000.0) / 1000.0;
        }

        // Function necessary to give correct value to robot (approximation to 0.**m so 1cm)
        void roundConfiguration(std::vector<double>& aConfig)
        {
            for (double& val : aConfig)
            {
                val = ceil(val*100.0)/100.0;
            }
        }
        
        // Function that perform the gripper moving
        void move_gripper(const std::vector<float>& gripper_joint_value, const float& seconds)
        {
            // Create an head controller action client to move the TIAGo's head
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client("/gripper_controller/follow_joint_trajectory", true);
            //ROS_INFO("Waiting for action server to start");
            // Wait for the action server to come up
            arm_client.waitForServer(); // Will wait for infinite time
            //ROS_INFO("Action server started, sending goal");

            // Send a goal to the action server
            control_msgs::FollowJointTrajectoryGoal aGoal = build_gripper_goal(gripper_joint_value, seconds);
            arm_client.sendGoal(aGoal);
            
            // Wait for the action to return
            bool finished_before_timeout = arm_client.waitForResult(ros::Duration(20.0));
            //bool finished_before_timeout = arm_client.waitForResult();
            
            // If the goal finished before the time out the goal status is reported
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = arm_client.getState();
                //ROS_INFO("Action Server finish: %s",state.toString().c_str());
            }
            // Otherwise the user is notified that the goal did not finish in the allotted time.
            else
            {
                //ROS_INFO("Action Server did not finish before the time out");
            }
        }

        // Generates a simple trajectory with one waypoints to move TIAGo's gripper 
        control_msgs::FollowJointTrajectoryGoal build_gripper_goal(const std::vector<float>& gripper_joint_value, const float& seconds)
        {
            control_msgs::FollowJointTrajectoryGoal goal;
            // The joint names, which apply to all waypoints
            goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
            goal.trajectory.joint_names.push_back("gripper_left_finger_joint");

            // Only one waypoints in this goal trajectory
            goal.trajectory.points.resize(1);

            // Trajectory point
            int index = 0;
            // Positions            
            goal.trajectory.points.at(index).positions.resize(2);
            // Velocities
            goal.trajectory.points.at(index).velocities.resize(2);
            for (int j = 0; j < gripper_joint_value.size(); ++j)
            {
                goal.trajectory.points.at(index).positions.at(j) = gripper_joint_value.at(j);
                goal.trajectory.points.at(index).velocities.at(j) = 0.0;
            }
            // To be reached 5 second after starting along the trajectory
            goal.trajectory.points[index].time_from_start = ros::Duration(seconds);
            return goal;
        }

        // Function to open the gripper
        void openGripper(const float& gripper_link_value, const float& seconds)
        {
            // Set them as open, wide enough for the object to fit
            std::vector<float> gripper_joint_value = {gripper_link_value, gripper_link_value};
            move_gripper(gripper_joint_value, seconds);
        }

        // Function to close the gripper
        void closedGripper(const float& gripper_link_value, const float& seconds)
        {
            // Set them as closed
            std::vector<float> gripper_joint_value = {gripper_link_value, gripper_link_value};
            move_gripper(gripper_joint_value, seconds);
        }

        // Function to virtually attach the object to the gripper
        bool attachGripper(const int& target_id)
        {
            ros::ServiceClient attach_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
            gazebo_ros_link_attacher::Attach attach_srv;
            // Set the names of the link to be attached and the object to be attached
            if(target_id == 1) // If ID == 1 then attach the blue hexagon
            {
                attach_srv.request.model_name_1 = "Hexagon";
                attach_srv.request.link_name_1 = "Hexagon_link";
            }
            else if(target_id == 2) // If ID == 2 then attach the green triangle
            {
                attach_srv.request.model_name_1 = "Triangle";
                attach_srv.request.link_name_1 = "Triangle_link";
            }
            else if(target_id == 3) // If ID == 3 then attach the red cube
            {
                attach_srv.request.model_name_1 = "cube";
                attach_srv.request.link_name_1 = "cube_link";
            }
            attach_srv.request.model_name_2 = "tiago";
            attach_srv.request.link_name_2 = "arm_7_link";
            // Call the link_attacher service
            if(attach_client.call(attach_srv))
            {
                ROS_INFO("Object successfully attached to the gripper");
                return true;
            }
            else
            {
                ROS_INFO("Failed to attach the object to the gripper");
                return false;
            }
        }

        // Function to detach the object from the gripper
        bool detachGripper(const int& target_id)
        {
            ros::ServiceClient detach_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
            gazebo_ros_link_attacher::Attach detach_srv;
            // Set the names of the link to be attached and the object to be attached
            if(target_id == 1) // If ID == 1 then detach the blue hexagon
            {
                detach_srv.request.model_name_1 = "Hexagon";
                detach_srv.request.link_name_1 = "Hexagon_link";
            }
            else if(target_id == 2) // If ID == 2 then detach the green triangle
            {
                detach_srv.request.model_name_1 = "Triangle";
                detach_srv.request.link_name_1 = "Triangle_link";
            }
            else if(target_id  == 3) // If ID == 3 then detach the red cube
            {
                detach_srv.request.model_name_1 = "cube";
                detach_srv.request.link_name_1 = "cube_link";
            }
            detach_srv.request.model_name_2 = "tiago";
            detach_srv.request.link_name_2 = "arm_7_link";
            // Call the link_attacher service
            if(detach_client.call(detach_srv))
            {
                ROS_INFO("Object successfully detached to the gripper");
                return true;
            }
            else
            {
                ROS_INFO("Failed to detach the object to the gripper");
                return false;
            }
        }

        // Function that compute the approach and the target pose wrt gripper (already setted)
        // All object
        void computeObjectApproachPosition_Y_angle(const geometry_msgs::Point& realPose, geometry_msgs::Point& approachPose, geometry_msgs::Point& graspPose, 
        const float& gripper_size, const float& distance, const float& aAngle_Y)
        {
            approachPose.x -= (cos(aAngle_Y) * (gripper_size + distance));
            approachPose.z += (sin(aAngle_Y) * (gripper_size + distance));

            graspPose.x -= (cos(aAngle_Y) * gripper_size);
            graspPose.z += (sin(aAngle_Y) * gripper_size);
        }

        // Function that perform linear movement wrt to caretsian coordinate, avoiding obstacles
        bool perform_linear_motion(moveit::planning_interface::MoveGroupInterface& group_arm, const geometry_msgs::Pose& initialPose, const geometry_msgs::Pose& targetPose)
        {
            group_arm.setStartState(*group_arm.getCurrentState());
            std::vector<geometry_msgs::Pose> aWaypoints;
            aWaypoints.push_back(initialPose);        
            aWaypoints.push_back(targetPose);
            group_arm.setPoseTarget(targetPose);

            // Plan the motion, reducing the maximum joint's velocities to perform the cartesian path
            moveit::planning_interface::MoveGroupInterface::Plan aPlan;
            moveit_msgs::RobotTrajectory aTrajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            //group_arm.setMaxVelocityScalingFactor(0.01); -> FOR BETTER PRECISION USE THIS
            group_arm.setMaxVelocityScalingFactor(0.5);
            group_arm.setMaxAccelerationScalingFactor(0.5);

            double fraction = group_arm.computeCartesianPath(aWaypoints, eef_step, jump_threshold, aTrajectory, true);
            if (fraction == -1.0)
            {
                ROS_INFO("No plan found!");
                return false;
            }
            else
            {
                ROS_INFO("Visualizing linear_plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
                aPlan.trajectory_ = aTrajectory;
                //  Execute the plan
                moveit::core::MoveItErrorCode error_code = group_arm.move();
                if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Motion completed!");
                    group_arm.setMaxVelocityScalingFactor(1.0);
                    group_arm.setMaxAccelerationScalingFactor(1.0);
                    return true;
                }
                else
                {
                    ROS_INFO("Motion failed! Error: %i:", error_code.val);
                    return false;
                }
            }
        }

        // Function that perform movement wrt to caretsian coordinate, avoiding obstacles
        bool perform_motion(moveit::planning_interface::MoveGroupInterface& group_arm, const geometry_msgs::Pose& aPose)
        {
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            group_arm.setStartState(*group_arm.getCurrentState());
            group_arm.setPoseTarget(aPose);
            moveit::core::MoveItErrorCode error_code = group_arm.plan(my_plan);
            if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
                //  Execute the plan
                error_code = group_arm.move();
                if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Motion completed!");
                    return true;
                }
                else
                {
                    ROS_INFO("Motion failed! Error: %i:", error_code.val);
                    return false;
                }
            }
            else
            {
                ROS_INFO("No plan found! Error: %i", error_code.val);
                return false;
            }            
        }

        // Function that perform movement wrt to joint values, avoiding obstacles
        bool perform_joint_motion(moveit::planning_interface::MoveGroupInterface& group_arm, const std::vector<double>& joint_group_positions)
        {
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            group_arm.setStartState(*group_arm.getCurrentState());
            group_arm.setJointValueTarget(joint_group_positions);
            moveit::core::MoveItErrorCode error_code = group_arm.plan(my_plan);
            if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
                //  Execute the plan
                error_code = group_arm.move();
                if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Motion completed!");
                    return true;
                }
                else
                {
                    ROS_INFO("Motion failed! Error: %i:", error_code.val);
                    return false;
                }
            }
            else
            {
                ROS_INFO("No plan found! Error: %i", error_code.val);
                return false;
            }
        }

        // Function to add cylindric tables to the collision object vector
        std::vector<std::string> addPlaceCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& group_arm,
        const geometry_msgs::Point& cylinder_target_point, geometry_msgs::Pose& place_pose)
        {
            std::vector<std::string> object_ids;

            // Creating environment
            // Create vector to hold 1 collision objects (place table)
            std::vector<moveit_msgs::CollisionObject> collision_objects;
            // 1 for the place table
            int number_objects = 1;
            collision_objects.resize(number_objects);
            ROS_INFO("Number of Collision object: %i", number_objects);

            // Define the id of the object, which is used to identify it.
            collision_objects[0].id = "cylinder";
            object_ids.push_back("cylinder");
            collision_objects[0].header.frame_id = group_arm.getPlanningFrame();

            float enlargment_cylinder_height = 0.06;
            float enlargment_cylinder_radius = 0.08;
            
            geometry_msgs::Pose cylinder_object_pose;
            cylinder_object_pose.position = cylinder_target_point;
            cylinder_object_pose.orientation = table_orientation;

            // Transform given costant and predifined value from map to base_footprint
            cylinder_object_pose = fixPose(cylinder_object_pose);
            cylinder_object_pose.position.z = (cylinder_height + enlargment_cylinder_height)/2.0;
            float cylinder_radius = cylinder_target_point.z;
            
            // Save the cylinder heigth to the real target pose -> where the object will be placed
            place_pose.position = cylinder_object_pose.position;
            place_pose.position.z = cylinder_height;

            // Define the primitive and its dimensions
            shape_msgs::SolidPrimitive primitive_cylinder;
            primitive_cylinder.type = primitive_cylinder.CYLINDER;

            // Define the primitive and its dimensions
            primitive_cylinder.dimensions.resize(2);
            primitive_cylinder.dimensions[0] = cylinder_height + enlargment_cylinder_height; // Height
            primitive_cylinder.dimensions[1] = cylinder_radius + enlargment_cylinder_radius; // Radius

            collision_objects[0].primitives.push_back(primitive_cylinder);
            collision_objects[0].primitive_poses.push_back(cylinder_object_pose);
            collision_objects[0].operation = collision_objects[0].ADD;

            // Now, add the collision object (cylinder place table) into the world
            planning_scene_interface.addCollisionObjects(collision_objects);
            // Return the list all the collision objects added, minus the target object     
            return object_ids;
        }

        // Function to add obstacles, objects and initial table to the collision object vector
        std::vector<std::string> addPickCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& group_arm, 
        const int& target_id, const geometry_msgs::Pose& target_pose, const float& target_size, 
        const std::vector<int>& obstacle_id, const std::vector<geometry_msgs::Pose>& obstacle_pose, const std::vector<float>& obstacle_size)
        {
            std::vector<std::string> object_ids;

            // Creating environment
            // Create vector to hold n+1 collision objects (+1 is the table)
            std::vector<moveit_msgs::CollisionObject> collision_objects;
            // 1 for the table and 1 for the target object to pick
            int number_objects = obstacle_id.size() + 1 + 1;
            collision_objects.resize(number_objects);
            ROS_INFO("Number of Collision object: %i", number_objects);

            // Define the id of the object, which is used to identify it.
            collision_objects[0].id = "table";
            object_ids.push_back("table");
            collision_objects[0].header.frame_id = group_arm.getPlanningFrame();

            // Define the primitive and its dimensions
            // Define the table (box) and add to the world.
            // Enlarge the table x,y position to avoid contact with it
            float enlargment_table = 0.25;
            float enlargment_table_z = 0.01;

            // Define the pose of the table (box)
            // It should be wrt to the center of the object
            // Transform given costant and predifined value from map to base_footprint
            geometry_msgs::Pose table_pose;
            table_pose.position = table_position;
            table_pose.orientation = table_orientation;
            table_pose = fixPose(table_pose);
            table_pose.position.z += enlargment_table_z/2.0;

            shape_msgs::SolidPrimitive primitive_table;
            primitive_table.type = primitive_table.BOX;
            primitive_table.dimensions.resize(3);
            primitive_table.dimensions[0] = table_surface_position.x + enlargment_table;
            primitive_table.dimensions[1] = table_surface_position.y + enlargment_table;
            primitive_table.dimensions[2] = table_surface_position.z + enlargment_table_z;

            collision_objects[0].primitives.push_back(primitive_table);
            collision_objects[0].primitive_poses.push_back(table_pose);
            collision_objects[0].operation = collision_objects[0].ADD;

            // Define the object that we will be manipulating
            collision_objects[1].header.frame_id = group_arm.getPlanningFrame();
            collision_objects[1].id = std::to_string(target_id);


            // Enlarge the the object to be picked on z to void contact with it
            // BLU
            float enlargment_blue_height = 0.1;
            float enlargment_red_height = 0.04;

            // Define the primitive and its dimensions
            shape_msgs::SolidPrimitive primitive_target;
            // Standard value, update for the specific object
            float target_heigth_z = target_pose.position.z - table_surface_position.z;
            float target_pose_z = table_surface_position.z + (target_heigth_z/2.0);     
            // Check if the target ID correspond to a red cube (3)
            if (target_id == 3)
            {
                target_heigth_z = target_pose.position.z + enlargment_red_height - table_surface_position.z;
                target_pose_z = table_surface_position.z + (target_heigth_z/2.0);
                //ROS_INFO("ID: %i is a red cube! Collision", target_id);
                primitive_target.type = primitive_target.BOX;
                primitive_target.dimensions.resize(3);
                primitive_target.dimensions[0] = target_size*2.0;
                primitive_target.dimensions[1] = target_size*2.0;
                primitive_target.dimensions[2] = target_heigth_z;
            }
            else
            {
                // Check if the target ID correspond to a blue hexagon (1)
                if (target_id == 1)
                {
                    //ROS_INFO("ID: %i is a blue hexagon! Collision", target_id);
                    primitive_target.type = primitive_target.CYLINDER;
                    target_heigth_z = target_pose.position.z + enlargment_blue_height - table_surface_position.z;
                    target_pose_z = table_surface_position.z + (target_heigth_z/2.0);
                    
                }
                // Check if the target ID correspond to a green triangle (2)
                else
                {
                    std::vector < std::string> a = {std::to_string(target_id)};
                    planning_scene_interface.removeCollisionObjects(a);
                    //ROS_INFO("ID: %i is a green triangle! Collision", target_id);
                    primitive_target.type = primitive_target.CONE;
                    target_heigth_z = target_pose.position.z + target_size - table_surface_position.z;
                    target_pose_z = table_surface_position.z;
                }
                // Define the primitive and its dimensions
                primitive_target.dimensions.resize(2);
                primitive_target.dimensions[0] = target_heigth_z; // Height
                primitive_target.dimensions[1] = target_size; // Radius
            }
            
            // Define the pose of the target object
            geometry_msgs::Pose target_object_pose;
            target_object_pose.position.x = target_pose.position.x;
            target_object_pose.position.y = target_pose.position.y;
            target_object_pose.position.z = target_pose_z;
            target_object_pose.orientation = target_pose.orientation;

            collision_objects[1].primitives.push_back(primitive_target);
            collision_objects[1].primitive_poses.push_back(target_object_pose);
            collision_objects[1].operation = collision_objects[1].ADD;

            // Define the collision objects for the obstacles to avoid
            // 30cm enlargment for heigth and 4cm for radius
            float enlargment_obstacle_height = 0.07;
            float enlargment_obstacle_radius = 0.02;
            for(int i = 2, j=0; i < number_objects; i++, j++)
            {
                collision_objects[i].header.frame_id = group_arm.getPlanningFrame();
                collision_objects[i].id = std::to_string(obstacle_id.at(j));
                object_ids.push_back(std::to_string(obstacle_id.at(j)));

                float obstacle_heigth_z = obstacle_pose.at(j).position.z - table_surface_position.z;
                float obstacle_pose_z = table_surface_position.z + (obstacle_heigth_z/2.0);

                // Define the primitive and its dimensions
                shape_msgs::SolidPrimitive primitive_obstacle;
                primitive_obstacle.type = primitive_obstacle.CYLINDER;
                primitive_obstacle.dimensions.resize(2);
                
                // Enlargment
                primitive_obstacle.dimensions[0] = obstacle_heigth_z + enlargment_obstacle_height; // Height
                primitive_obstacle.dimensions[1] = obstacle_size.at(j) + enlargment_obstacle_radius; // Radius

                // Define the pose of the obstacles
                geometry_msgs::Pose obstacle_object_pose;
                obstacle_object_pose.position.x = obstacle_pose.at(j).position.x;
                obstacle_object_pose.position.y = obstacle_pose.at(j).position.y;
                obstacle_object_pose.position.z = obstacle_pose_z;
                obstacle_object_pose.orientation = obstacle_pose.at(j).orientation;

                collision_objects[i].primitives.push_back(primitive_obstacle);
                collision_objects[i].primitive_poses.push_back(obstacle_object_pose);
                collision_objects[i].operation = collision_objects[i].ADD;
            }

            // Now, add the collision objects into the world
            planning_scene_interface.addCollisionObjects(collision_objects);
            // Return the list all the collision objects added, minus the target object     
            return object_ids;
        }

        // Function to perform the pick routine
        bool pick_object(const assignment_2_group_27::PickPlaceGoalConstPtr& goal)
        {
            std::vector<std::string> collision_objects_ids;
            //r.sleep();
            ros::WallDuration(1.0).sleep();
            const robot_state::JointModelGroup* joint_model_group = group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            

            // 3. Define a collision object for every object you detect and for the table.
            // Configure the collision objects
            ROS_INFO("Define collision objects");
            feedback_.status = "Define collision objects";
            r.sleep();
            collision_objects_ids = addPickCollisionObjects(planning_scene_interface, group_arm, 
            goal->marker_ID, goal-> marker_pose, goal-> marker_size,
            goal->obs_ID, goal->obs_pose, goal->obs_size);

            if (collision_objects_ids.size() == 0)
            {
                ROS_INFO("Fail to addPickCollisionObjects");
                return  false;
            }
            // Wait a bit for ROS things to initialize
            r.sleep();

            // 4. Assign an initial configuration to the Tiago’s arm. With MoveIt you can move only the arm or both torso and arm
            ROS_INFO("Assign initial configuration to the arm");
            feedback_.status = "Assign initial configuration to the arm";
            r.sleep();
            moveit::core::RobotStatePtr current_state = group_arm.getCurrentState();
            std::vector<double> initial_joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, initial_joint_group_positions);
            std::vector<std::string> joint_names = group_arm.getJointNames();
            roundConfiguration(initial_joint_group_positions);
            
            
            // 5. Make the arm move in a target position. 
            // The target position is set at 10 cm before for blue hexagon (front approach)
            // Otherwise target position is set at 5 cm above (top approach)
            ROS_INFO("Move arm to target position");
            feedback_.status = "Move arm to target position";
            r.sleep();

            geometry_msgs::Pose real_marker_pose = goal-> marker_pose;
            geometry_msgs::Pose intermediate_pose;
            geometry_msgs::Pose approach_pose;
            geometry_msgs::Pose grasp_pose;

            // If the object to be picked is the blue hexagon, then perform a front approach
            // Otherwise perform an approach from the top
            if (goal-> marker_ID == 1)
            {
                // Compute offset wrt detected object
                RPY_angle blue_off = computeOffset(real_marker_pose.orientation);
                grasp_pose.orientation = make_quaternion_from_xyz(blue_pick_orientation);
                approach_pose.orientation = grasp_pose.orientation;

                // Compute the center of blue hexagon + safe_padding to not collide with table 
                float z_pose = (real_marker_pose.position.z - table_surface_position.z)/4.0;
                real_marker_pose.position.z -= z_pose;

                approach_pose.position = real_marker_pose.position;
                grasp_pose.position = real_marker_pose.position;

                // Compute the grasp pose and the approach pose
                // approachPose is 10cm before the marker position
                computeObjectApproachPosition_Y_angle(real_marker_pose.position, approach_pose.position, grasp_pose.position, x_gripper_offset, 0.1, blue_pick_orientation.pitch);
                
                // Move outside of the table on the left wrt to base_footprint, check the first line for the value
                intermediate_pose.position = left_intermediate_point;
                intermediate_pose.orientation = left_intermediate_orientation;
            }
            else
            {
                // If the object to be picked is the green triangle (2)
                if (goal-> marker_ID == 2)
                {
                    RPY_angle green_off = computeOffset(real_marker_pose.orientation);
                    // If the angle is not valid try to use standard value -> green_z_angle (-PI/7)
                    green_pick_orientation.yaw = green_off.yaw-M_PI/2.0;
                    grasp_pose.orientation = make_quaternion_from_xyz(green_pick_orientation);
                    approach_pose.orientation = grasp_pose.orientation;

                    // Compute the center of green triangle + its size + safe_padding to not collide with table
                    float z_pose = goal->marker_size;
                    real_marker_pose.position.z += z_pose;
                    // Compute the displacement wrt the rotation on angle z -> YAW
                    real_marker_pose.position.x += cos(green_pick_orientation.yaw)*goal->marker_size;
                    real_marker_pose.position.y += sin(green_pick_orientation.yaw)*goal->marker_size/2.0;

                    approach_pose.position = real_marker_pose.position;
                    grasp_pose.position = real_marker_pose.position;

                    // Compute the grasp pose and the approach pose
                    // approachPose is 5cm on top of the marker position
                    //computeObjectApproachPosition_Y_angle(real_marker_pose.position, approach_pose.position, grasp_pose.position, x_gripper_offset, 0.05, green_pick_orientation.pitch);
                    computeObjectApproachPosition_Y_angle(real_marker_pose.position, approach_pose.position, grasp_pose.position, x_gripper_offset, 0.05, green_pick_orientation.pitch);
                }
                // If the object to be picked is the red cube (3)
                else
                {
                    RPY_angle red_off = computeOffset(real_marker_pose.orientation);
                    // If the angle is not valid try to use standard value -> 0.0
                    red_pick_orientation.yaw = red_off.yaw;
                    grasp_pose.orientation = make_quaternion_from_xyz(red_pick_orientation);
                    approach_pose.orientation = grasp_pose.orientation;
                    
                    // Use the top of red cube as grasp pose
                    approach_pose.position = real_marker_pose.position;
                    grasp_pose.position = real_marker_pose.position;

                    // Compute the grasp pose and the approach pose
                    // approachPose is 5cm before the marker position
                    computeObjectApproachPosition_Y_angle(real_marker_pose.position, approach_pose.position, grasp_pose.position, x_gripper_offset, 0.05, red_pick_orientation.pitch);
                }
                // Move outside of the table on the rigth wrt to base_footprint, check the first line for the value
                intermediate_pose.position = right_intermediate_point;
                intermediate_pose.orientation = right_intermediate_orientation;
            }          

            // Save this value which is the contact point on z wrt to object
            // This is the offset on z axis (base_footprint frane) wrt to the table surface.
            // This value is used to make a more precise place
            // Since the green triangle has the marker of its structure, we compute the real offset wrt top of triangle
            if (goal-> marker_ID == 2)
            {
                result_.z_object_pick_contact_point = real_marker_pose.position.z + goal->marker_size - table_surface_position.z;
            }
            else
            {
                result_.z_object_pick_contact_point = real_marker_pose.position.z - table_surface_position.z;
            }
            
            ROS_INFO("z_object_pick_contact_point: %f", result_.z_object_pick_contact_point);
            
            // Round the given value to 1mm precision
            roundPoint(grasp_pose.position);
            roundPoint(approach_pose.position);
            roundPoint(intermediate_pose.position);
            

            //FOR DEBUGGING IN CASE OF ERROR
            
            ROS_INFO("real_marker_pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
            real_marker_pose.position.x, real_marker_pose.position.y, real_marker_pose.position.z,
            real_marker_pose.orientation.x, real_marker_pose.orientation.y, real_marker_pose.orientation.z, real_marker_pose.orientation.w);

            ROS_INFO("intermediate_pose pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
            intermediate_pose.position.x, intermediate_pose.position.y, intermediate_pose.position.z,
            intermediate_pose.orientation.x, intermediate_pose.orientation.y, intermediate_pose.orientation.z, intermediate_pose.orientation.w);
            
            ROS_INFO("approach_pose pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
            approach_pose.position.x, approach_pose.position.y, approach_pose.position.z,
            approach_pose.orientation.x, approach_pose.orientation.y, approach_pose.orientation.z, approach_pose.orientation.w);

            ROS_INFO("grasp_pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
            grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
            grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
            
            
            // 5. Make the arm move in a target position
            ROS_INFO(" performing approach_pose");
            if (! perform_motion(group_arm, approach_pose))
            {
                ROS_INFO("Fail to reach approach_pose");
                return  false;
            }
            r.sleep();
            
            // 6. Through a linear movement complete the grasping. This sub-sequence from 4-6
            // should help you to implement the correct grasping of the objects (Figure 5b).
            ROS_INFO("Complete the grasping");
            feedback_.status = "Complete the grasping";
            if (! perform_linear_motion(group_arm, approach_pose, grasp_pose))
            {
                ROS_INFO("Fail to reach linear grasp_pose");
                return  false;
            }
            r.sleep();

            // 7. Remove the target object from the collision objects
            ROS_INFO("Remove object from the collision objects");
            feedback_.status = "Remove object from the collision objects";
            std::vector<std::string> object_ids;
            object_ids.push_back(std::to_string(goal->marker_ID));
            planning_scene_interface.removeCollisionObjects(object_ids);
            r.sleep();

            // 8. Attach virtually the object to the gripper (use arm_7_link) using the appropriate Gazebo plugin: Gazebo_ros_link_attacher
            ROS_INFO("Attach virtually the object to the gripper");
            feedback_.status = "Attach virtually the object to the gripper";
            if ( ! attachGripper(goal->marker_ID))
            {
                ROS_INFO("Failed to attach virtually the object to the gripper");
                return  false;
            }
            r.sleep();

            // 9. Close the gripper.
            ROS_INFO("Close the gripper");
            feedback_.status = "Close the gripper";
            // 0.00 for closedGripper
            closedGripper(0.00, 1.0);
            r.sleep();

            // 10. Come back to the previous target position through a linear movement of the arm.
            ROS_INFO("Return to target position");
            feedback_.status = "Return to target position";
            if (! perform_linear_motion(group_arm, grasp_pose, approach_pose))
            {
                ROS_INFO("Fail to reach linear approach_pose");
                return  false;
            }
            r.sleep();

            // 11. Move the arm to an intermediate pose (Figure 5c).
            ROS_INFO("Move the arm to an intermediate pose");
            feedback_.status = "Move the arm to an intermediate pose";
            if (! perform_motion(group_arm, intermediate_pose))
            {
                ROS_INFO("Fail to reach intermediate_pose");
                return  false;
            }
            r.sleep();
            
            // 12. Move the arm to a secure pose to avoid collisions before navigate to the destination 
            // (i.e. fold the robot’s arm close to the robot’s body).
            ROS_INFO("Move arm to a secure pose");
            feedback_.status = "Move arm to a secure pose";
            if (! perform_joint_motion(group_arm, initial_joint_group_positions))
            {
                ROS_INFO("Fail to reach initial_joint_group_positions");
                return  false;
            }
            r.sleep();

            // Remove all collision object
            ROS_INFO("Remove all collision object");
            planning_scene_interface.removeCollisionObjects(collision_objects_ids);
            r.sleep();
            return true;
        }

        // Function to perform the place routine
        bool place_object(const assignment_2_group_27::PickPlaceGoalConstPtr& goal)
        {
            std::vector<std::string> collision_objects_ids;
            r.sleep();
            
            const robot_state::JointModelGroup* joint_model_group = group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

            // 3. Define a collision object for cylinder table.
            // Configure the collision object
            ROS_INFO("Define collision objects");
            feedback_.status = "Define collision objects";
            r.sleep();
            // Get also the place_pose = [x,y: center of cylinder] [z: height]
            // [x,y]: center of cylinder - (cylinder_radius+enlargment)/2.0 wrt base_footprint
            // [z]: height of cylinder wrt base_footprint
            geometry_msgs::Pose real_place_pose; // actual pose for place
            collision_objects_ids = addPlaceCollisionObjects(planning_scene_interface, group_arm,
            goal->cylinder_place_point, real_place_pose);
            // Wait a bit for ROS things to initialize
            r.sleep();

            // 4. Assign an initial configuration to the Tiago’s arm.
            ROS_INFO("Assign initial configuration");
            feedback_.status = "Assign initial configuration";
            r.sleep();
            moveit::core::RobotStatePtr current_state = group_arm.getCurrentState();
            std::vector<double> initial_joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, initial_joint_group_positions);
            std::vector<std::string> joint_names = group_arm.getJointNames();
            roundConfiguration(initial_joint_group_positions);

            // EXTRA POINT
            // Place the object 10cm on top of the cylinder wrt cylinder height (0.69m) + z_offset wrt contact point
            // z_offset was defined in the pick part
            real_place_pose.position.z += 0.07 + goal->z_object_place_contact_point;
            
            ROS_INFO("z_object_place_contact_point: %f", goal-> z_object_place_contact_point);
            geometry_msgs::Pose place_pose;
            geometry_msgs::Pose place_approach_pose;
            place_pose.position = real_place_pose.position;
            float pitch;
            // Green and Red obstacle (top approach)
            if (goal->marker_ID == 2 || goal->marker_ID == 3)
            {
                place_pose.orientation = make_quaternion_from_xyz(place_top_orientation);
                place_approach_pose = place_pose;
                // Compute the grasp pose and the approach pose
                // approachPose is 5cm before the marker position
                computeObjectApproachPosition_Y_angle(real_place_pose.position, place_approach_pose.position, place_pose.position, x_gripper_offset, 0.05, place_top_orientation.pitch);
                // Move back of 10cm wrt x axis of base_footprint  
            }
            // Blue obstacle (front approach)
            else
            {
                real_place_pose.position.z += 0.02;
                place_pose.orientation = make_quaternion_from_xyz(place_front_orientation);
                place_approach_pose = place_pose;
                // Compute the grasp pose and the approach pose
                // approachPose is 5cm before the marker position
                computeObjectApproachPosition_Y_angle(real_place_pose.position, place_approach_pose.position, place_pose.position, x_gripper_offset, 0.05, place_front_orientation.pitch);
                // Move back of 10cm wrt x axis of base_footprint
                place_approach_pose.position;
            }

            // Round the given pose to 1mm precision
            roundPoint(place_pose.position);

            // FOR DEBUGGING IN CASE OF ERROR
            
            ROS_INFO("real_place_pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
            real_place_pose.position.x, real_place_pose.position.y, real_place_pose.position.z,
            real_place_pose.orientation.x, real_place_pose.orientation.y, real_place_pose.orientation.z, real_place_pose.orientation.w);

            ROS_INFO("place_pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", 
            place_pose.position.x, place_pose.position.y, place_pose.position.z,
            place_pose.orientation.x, place_pose.orientation.y, place_pose.orientation.z, place_pose.orientation.w);
            
            // 14. Place the object on the top of the table        
            ROS_INFO("place_pose");
            if (! perform_motion(group_arm, place_pose))
            {
                ROS_INFO("Fail to reach place_pose");
                return  false;
            }
            r.sleep();

            // 15. Open the gripper.
            // 0.04 for close the gripper
            ROS_INFO("Open the gripper");
            feedback_.status = "Open the gripper";
            r.sleep();
            openGripper(0.04, 1.0);
            r.sleep();

            // 16. Detach the object via the Gazebo plugin.
            ROS_INFO("Detaching the object from the gripper");
            feedback_.status = "Detaching the object from the gripper";
            r.sleep();
            if ( ! detachGripper(goal->marker_ID))
            {
                ROS_INFO("Failed to detach virtually the object to the gripper");
                return  false;
            }
            r.sleep();
            
            ROS_INFO("Return to a safe pase");            
            ROS_INFO("initial_joint_group_positions");
            if (! perform_joint_motion(group_arm, initial_joint_group_positions))
            {
                ROS_INFO("Fail to reach initial_joint_group_positions");
                return  false;
            }
            r.sleep();

            // Remove cylindrical place table
            ROS_INFO("Remove all collision object");
            planning_scene_interface.removeCollisionObjects(collision_objects_ids);
            return true;
        }
    
        // Function that initialize the robot configuration
        void initial_tiago_configuration()
        {
            // Select group of joints
            group_arm.setPoseReferenceFrame(target_frame);
            group_arm.setMaxVelocityScalingFactor(1.0);
            group_arm.setPlannerId("SBLkConfigDefault");
            group_arm.setPlanningTime(45.0);
            const robot_state::JointModelGroup* joint_model_group = group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            ROS_INFO("Planning frame: %s", group_arm.getPlanningFrame().c_str());
            ROS_INFO("End effector link: %s", group_arm.getEndEffectorLink().c_str());
            ROS_INFO("Available Planning Groups:"); std::copy(group_arm.getJointModelGroupNames().begin(), group_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
        }

    public:        
        PPAction(std::string name, std::string planning_group) : as_(nh_, name, boost::bind(&PPAction::PickPlace, this, _1), false), action_name_(name),
        group_arm(planning_group)
        {
            as_.start();
            PLANNING_GROUP = planning_group;
        }

        ~PPAction(void)
        {
        }

        void PickPlace(const assignment_2_group_27::PickPlaceGoalConstPtr& goal)
        {
            // Variable that return if the pick or place is completed
            bool success = false;
            // Publish info to the console for the user
            ROS_INFO("%s: Executing", action_name_.c_str());
            
            // Initialize only the first time that is called
            if (initialized == false)
            {
                initial_tiago_configuration();
                initialized = true;
            }

            // Perform pick
            if (goal->pick == true && goal->place == false)
            {
                ROS_INFO("The server just receive the target AprilTag marker %i from node_a", (int) goal->marker_ID);
                feedback_.status = "The server just receive the target AprilTag marker " + std::to_string(goal->marker_ID) + " from node_a";
                // Publish the feedback
                as_.publishFeedback(feedback_);
                r.sleep();

                ROS_INFO("Start pick object");
                feedback_.status = "Start pick object";
                // Publish the feedback
                as_.publishFeedback(feedback_);
                r.sleep();
                success = pick_object(goal);
            }
            // Perform place
            else if (goal->pick == false && goal->place == true)
            {
                ROS_INFO("The server just receive the target AprilTag marker %i from node_a", (int) goal->marker_ID);
                feedback_.status = "The server just receive the target AprilTag marker " + std::to_string(goal->marker_ID) + " from node_a";
                // Publish the feedback
                as_.publishFeedback(feedback_);
                ros::WallDuration(1.0).sleep();
                //r.sleep();
                ROS_INFO("Start place object");
                feedback_.status = "Start place object";
                // Publish the feedback
                as_.publishFeedback(feedback_);
                //r.sleep();
                success = place_object(goal);
            }
            // Error
            else
            {
                ROS_INFO("Goal is not correct");
                feedback_.status = "Goal is not correct";
                as_.publishFeedback(feedback_);
                ROS_INFO("%s: Aborted", action_name_.c_str());
                // Set the action state to aborted
                as_.setAborted(result_);
            }

            
            // Check that preempt has not been requested by the client
            if(!as_.isActive() || as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                as_.setAborted(result_);
                return;
            }

            // Set the result of the action and return it to the action client (node_a),
            // if the target object is picked/placed then return a succesfull response
            if (success)
            {
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
            else
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                as_.setAborted(result_);
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_c");
    // "arm_torso" is our spoecific configuration
    // With MoveIt you can move only the arm ()"arm") or both torso and arm ("arm_torso")
    PPAction pick_place("node_c", "arm_torso");
    ros::spin();
    return 0;
}
/**
 * Group 27
 * Luca Sambin
 * Andrea Fonsato
 * Zhaku Bejaj
*/

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <assignment_2_group_27/DetectionAction.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class DetectionAction
{
    private:
        // Function that stores in a pointer the messages published in the /tag_detections topic by the apriltag node
        bool updateArrayTags(apriltag_ros::AprilTagDetectionArrayConstPtr& aTag_array_msg) 
        {
            aTag_array_msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh_, ros::Duration(5.0));
            if (aTag_array_msg != NULL)
            {
                return true;
            }
            else
            {
                return false;
            }           
        }

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

        // Function that performs the head movement, the 1st argument is for left/right moves, while the 2nd argument represents up/down movements.
        // The purpose of this function is to send  trajectory goals to the controller. The goal is generated in the "build_head_goal" function.
        // The values of the 2 arguments are in the interval [-1,24, 1,24] (right/left) and [-0,98, 0,72] (down/up).
        // Negative values represent right/down move respectively, while positive values represent left/up moves respectively
        bool move_head(const float& head_1_joint, const float& head_2_joint)
        {
            // Create an head controller action client to move the TIAGo's head
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_client("/head_controller/follow_joint_trajectory", true);
            ROS_INFO("Waiting for action server to start");
            // Wait for the action server to come up
            head_client.waitForServer(); // Will wait for infinite time
            ROS_INFO("Action server started, sending goal");

            // Send a goal to the action server
            control_msgs::FollowJointTrajectoryGoal aGoal = build_head_goal(head_1_joint, head_2_joint);
            head_client.sendGoal(aGoal);
            
            // Wait for the action to return
            bool finished_before_timeout = head_client.waitForResult(ros::Duration(20.0));
            
            // If the goal finished before the time out the goal status is reported
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = head_client.getState();
                ROS_INFO("Action Server finish: %s",state.toString().c_str());
                return true;
            }
            // Otherwise the user is notified that the goal did not finish in the allotted time.
            else
            {
                ROS_INFO("Action Server did not finish before the time out");
                return false;
            }
        }

        // Generates a simple trajectory with one waypoints to move TIAGo's head
        control_msgs::FollowJointTrajectoryGoal build_head_goal(const float& head_1_joint, const float& head_2_joint)
        {
            control_msgs::FollowJointTrajectoryGoal goal;
            // The joint names, which apply to all waypoints
            goal.trajectory.joint_names.push_back("head_1_joint");
            goal.trajectory.joint_names.push_back("head_2_joint");

            goal.trajectory.points.resize(1);
            // First trajectory point
            int index = 0;
            // Positions      
            goal.trajectory.points.at(index).positions.resize(2);
            goal.trajectory.points.at(index).positions.at(0) = head_1_joint;
            goal.trajectory.points.at(index).positions.at(1) = head_2_joint;
            // Velocities
            goal.trajectory.points.at(index).velocities.resize(2);
            for (int j = 0; j < 2; ++j)
            {
                goal.trajectory.points.at(index).velocities.at(j) = 0.0;
            }
            // To be reached 6 second after starting along the trajectory
            goal.trajectory.points.at(index).time_from_start = ros::Duration(3.0);
            return goal;
        }

        // Function that performs the torso movement
        // The purpose of this function is to send  trajectory goals to the controller. 
        //The goal is generated in the "build_torso_goal" function.
        bool move_torso(const float& torso_joint)
        {
            // Create a torso controller action client to move the TIAGo's torso
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client("/torso_controller/follow_joint_trajectory", true);
            ROS_INFO("Waiting for action server to start");
            // torso_client for the action server to come up
            torso_client.waitForServer(); // Will wait for infinite time
            ROS_INFO("Action server started, sending goal");

            // Send a goal to the action server
            control_msgs::FollowJointTrajectoryGoal aGoal = build_torso_goal(torso_joint);
            torso_client.sendGoal(aGoal);
            
            // Wait for the action to return
            bool finished_before_timeout = torso_client.waitForResult(ros::Duration(20.0));
            
            // If the goal finished before the time out the goal status is reported
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = torso_client.getState();
                ROS_INFO("Action Server finish: %s",state.toString().c_str());
                return true;
            }
            // Otherwise the user is notified that the goal did not finish in the allotted time.
            else
            {
                ROS_INFO("Action Server did not finish before the time out");
                return false;
            }
        }

        // Generates a simple trajectory with one waypoint to move TIAGo's torso
        control_msgs::FollowJointTrajectoryGoal build_torso_goal(const float& torso_joint)
        {
            control_msgs::FollowJointTrajectoryGoal goal;
            // The joint name, which apply to waypoint
            goal.trajectory.joint_names.push_back("torso_lift_joint");

            goal.trajectory.points.resize(1);
            // First trajectory point
            int index = 0;
            // Positions      
            goal.trajectory.points.at(index).positions.resize(1);
            goal.trajectory.points.at(index).positions.at(0) = torso_joint;
            // Velocities
            goal.trajectory.points.at(index).velocities.resize(1);
            for (int j = 0; j < 1; ++j)
            {
                goal.trajectory.points.at(index).velocities.at(j) = 0.0;
            }
            // To be reached 3 second after starting along the trajectory
            goal.trajectory.points.at(index).time_from_start = ros::Duration(3.0);
            return goal;
        }

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_2_group_27::DetectionAction> as_;
        std::string action_name_;
        assignment_2_group_27::DetectionResult result_;
        assignment_2_group_27::DetectionFeedback feedback_;
        // Definition of origin head joint value (look straight)
        const float origin_head_1_joint = 0.00;
        const float origin_head_2_joint = 0.00;

    public:
        DetectionAction(std::string name): as_(nh_, name, boost::bind(&DetectionAction::aprilCallback,this,_1),false), action_name_(name)
        {
            as_.start();
        }

        ~DetectionAction(void) {}     

        // Define the aprilCallback(), function that will be called when the action server receives a goal
        void aprilCallback(const assignment_2_group_27::DetectionGoalConstPtr &goal)
        {
            // Helper variables
            ros::Rate r(1);
            // Publish info to the console for the user
            ROS_INFO("%s: Executing", action_name_.c_str());

            // Clear previuos values
            // "marker_" field denote information about the tag of the object we currently want to pick
            // "obs_" field denote information about the obstacles on the table
            result_.marker_ID = 0;
            result_.marker_size = 0.0;
            result_.obs_pose.clear();
            result_.obs_ID.clear();
            result_.obs_size.clear();

            // Set to true when we detect the goal tag
            bool marker_id_detected = false;
            // Variable that stores the array of detections given by the apriltag node
            apriltag_ros::AprilTagDetectionArrayConstPtr tag_array_msg = NULL;
            tf::TransformListener listener;
            tf::StampedTransform transform;
            

            // The server just receive the targer AprilTag marker from node_a
            ROS_INFO("The server just receive the target AprilTag marker %i from node_a", (int) goal->marker_ID);
            feedback_.status = "The server just receive the target AprilTag marker " + std::to_string(goal->marker_ID) + " from node_a";
            // Publish the feedback
            as_.publishFeedback(feedback_);
            r.sleep();

            ROS_INFO("Torso is moving");
            feedback_.status = "Torso is moving";
            as_.publishFeedback(feedback_);
            // If the controller doesn't reach the waypoint set action state as aborted   
            if (! move_torso(0.34))
            {
                ROS_INFO("Torso was not moved correctly");
                feedback_.status = "Torso was not moved correctly";
                // Publish the feedback
                as_.publishFeedback(feedback_);
                ROS_INFO("%s: Aborted", action_name_.c_str());
                // Set the action state to aborted
                as_.setAborted(result_);
                return;
            }    

            // Moving the head before starting the detection of the markers
            // Set the current status of the robot in the feedback message and publish it
            ROS_INFO("Head is moving");
            feedback_.status = "Head is moving";
            as_.publishFeedback(feedback_);    
            // If the controller doesn't reach the waypoint set action state as aborted   
            if (! move_head(goal->head_1_joint, goal->head_2_joint))
            {
                ROS_INFO("Head was not moved correctly");
                feedback_.status = "Head was not moved correctly";
                // Publish the feedback
                as_.publishFeedback(feedback_);
                ROS_INFO("%s: Aborted", action_name_.c_str());
                // Set the action state to aborted
                as_.setAborted(result_);
                return;
            }
            
            // Check that preempt has not been requested by the client
            if(!as_.isActive() || as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                // Set the action state to aborted
                as_.setAborted(result_);
                return;
            }

            // Wait 3s before performing the detection, so the head of Tiago is fully stopped
            r.sleep();
            r.sleep();
            r.sleep();

            // Set the current status of the robot in the feedback message and publish it
            ROS_INFO("Starting the detection of the markers");
            feedback_.status = "Starting the detection of the markers";
            // Publish the feedback
            as_.publishFeedback(feedback_);
            
            // We will wait until we receive some pointer from the /tag_detections topic
            while (! updateArrayTags(tag_array_msg)); 

            // We want the transform from the camera frame "xtion_rgb_optical_frame" to the base frame "base_footprint"
            // Define the target frame (base_footprint) and source frame (xtion_rgb_optical_frame)
            std::string target_frame = "base_footprint";
            std::string source_frame = tag_array_msg->header.frame_id.c_str(); // "xtion_rgb_optical_frame"
            try
            {
                listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
                listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
                feedback_.status = "Transformation is failed!";
                // Publish the feedback
                as_.publishFeedback(feedback_);
                ROS_INFO("%s: Aborted", action_name_.c_str());
                // Set the action state to aborted
                as_.setAborted(result_);
                return;
            }
            
            // Look for marker(tag of the object we want to pick) into the array of detections
            for(int i=0; i < tag_array_msg->detections.size(); i++)
            {
                 //Check if we found the goal tag at the current position in the array of detections
                if (tag_array_msg->detections.at(i).id.at(0) == goal->marker_ID)
                {
                    // Save the marker ID, size, pose
                    result_.marker_ID = tag_array_msg->detections.at(i).id.at(0);
                    result_.marker_size = tag_array_msg->detections.at(i).size.at(0);
                    result_.marker_pose = transformPose(tag_array_msg->detections.at(i).pose.pose.pose, transform);
                    marker_id_detected = true;
                }
                else                
                {
                    // Save the obstacle ID, size, pose
                    result_.obs_ID.push_back(tag_array_msg->detections.at(i).id.at(0));
                    result_.obs_size.push_back(tag_array_msg->detections.at(i).size.at(0));
                    geometry_msgs::Pose tmp = transformPose(tag_array_msg->detections.at(i).pose.pose.pose, transform);
                    result_.obs_pose.push_back(tmp);
                }

                // Print the values of the marker
                ROS_INFO("Position of marker %i: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]", tag_array_msg->detections.at(i).id.at(0),
                tag_array_msg->detections.at(i).pose.pose.pose.position.x, tag_array_msg->detections.at(i).pose.pose.pose.position.y,
                tag_array_msg->detections.at(i).pose.pose.pose.position.z, tag_array_msg->detections.at(i).pose.pose.pose.orientation.x, 
                tag_array_msg->detections.at(i).pose.pose.pose.orientation.y, tag_array_msg->detections.at(i).pose.pose.pose.orientation.z, 
                tag_array_msg->detections.at(i).pose.pose.pose.orientation.w);
            }           

            // Set the result of the action and return it to the action client (node_a),
            // if the target AprilTag marker is detected then return a succesfull response
            if (marker_id_detected)
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
    ros::init(argc,argv,"node_b");
    DetectionAction detection("node_b");
    ros::spin();
    return 0;
}
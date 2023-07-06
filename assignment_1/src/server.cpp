/**
 * Group 27
 * Luca Sambin
 * Andrea Fonsato
 * Zhaku Bejaj
*/

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <assignment_1_group_27/TiagoAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

class TiagoAction
{
    private:
        // Struct that we use to store Point Sets
        struct PointSet
        {
            cv::Point2f first_point;
            cv::Point2f middle_point;
            cv::Point2f last_point;
            std::vector<cv::Point2f> points;
        };

        // Struct that we use to store Circles
        struct Circle
        {
            cv::Point2f center;
            float radius;
        };

        // Minimum number of points that make up a group that will be processed later
        const int min_group_points = 5;
        // Maximum distance between two points of the same set
        const float max_group_distance = 0.15;
        // Variable that stores the maximum allowed radius value for a circle
        const float max_radius = 0.25;

        // Varaible that we use to store 
        std::vector<PointSet> my_point_sets;
        std::vector<Circle> my_circles;
        std::vector<cv::Point2f> my_input_points;

        // Function that calculate the distance of a point to its origin
        float length(const cv::Point2f& aPoint)
        {
            return sqrt(pow(aPoint.x, 2.0) + pow(aPoint.y, 2.0));
        }

        // Function called every time that we need to find the obstacle given the data from the laser scan
        void obstacle_detection(const sensor_msgs::LaserScanConstPtr& aLaser_msg)
        {
            // Clear the previous results
            my_input_points.clear();
            my_point_sets.clear();
            my_circles.clear();

            // The first and the last 19 measure are invalid, so we skip them. The robot sees itself.
            // Convert each readout of the laser range finder into [x, y] coordinates in the reference frame of the laser
            for (int i=19; i < aLaser_msg->ranges.size()-18; i++) 
            {
                float r = aLaser_msg->ranges.at(i);
                // Extract all the position from the laser scan and convert into [x,y] position,
                // only if inside the range given by the laser
                if (r >= aLaser_msg->range_min && r <= aLaser_msg->range_max)
                {
                    float phi = aLaser_msg->angle_min + (i * aLaser_msg->angle_increment);
                    my_input_points.push_back(cv::Point2f(r * cos(phi), r * sin(phi)));
                }
            }

            // Grouping points and simultaneously detects PointSet
            groupPoints(aLaser_msg->angle_increment);
            // Detect a circle for each PointSet. 
            // We also check if the detected cylindric obstacles, are valid (circle) or are invalid (lines, corners, ...)
            detectCircles();
        }
        

        void groupPoints(const float& angle_increment)
        {
            // Helper variables, used to iterate over the all ranges given by the laser scanner
            int start_index = 0;
            int end_index = 0;
            cv::Point2f point_start = my_input_points.at(0);
            cv::Point2f point_end = my_input_points.at(0);
            int num_points = 1;
            float distance_proportion = angle_increment;

            for (int i=1; i < my_input_points.size(); i++)
            {
                // Calculate the distance of the actual point 
                float range = length(my_input_points.at(i));
                float distance = length(my_input_points.at(i) - point_end);

                // If the distance between two points is greater than this value, start a new group
                // otherwise update the end point, its index and the number of points inside this set
                if (distance < max_group_distance + range * distance_proportion)
                {
                    point_end = my_input_points.at(i);
                    num_points++;
                    end_index = i;
                }
                // Store the actual point set (only if number of point is > 5) and then start a new set of points
                else
                {
                    // Store the actual point set (only if number of point is > 5)
                    detectPointSet(point_start, point_end, start_index, end_index, num_points);

                    // Start a newset of points
                    start_index = i;
                    end_index = i;
                    point_start = my_input_points.at(i);
                    point_end = my_input_points.at(i);                    
                    num_points = 1;
                }
            }

            // Check the last point set too!
            detectPointSet(point_start, point_end, start_index, end_index, num_points);
        }

        // Store the actual point set inside our variable that will be processed later
        void detectPointSet(cv::Point2f& point_start, cv::Point2f& point_end, int& start_index, int& end_index, int& num_points)
        {
            // Number of points is not sufficient
            if (num_points < min_group_points)
            {
                return;
            }

            // Store the point set
            PointSet tmp_ptSet;
            tmp_ptSet.first_point = point_start;
            tmp_ptSet.middle_point = my_input_points.at(start_index+((end_index-start_index)/2));
            tmp_ptSet.last_point = point_end;
            for (int i=start_index; i < my_input_points.size() && i <= end_index; i++)
            {
                tmp_ptSet.points.push_back(my_input_points.at(i));
            }
            // Add the point set to our internal variable
            my_point_sets.push_back(tmp_ptSet);
        }

        // Store the cylindrical obstacle position inside our variable
        void detectCircles() 
        {
            for (const PointSet& ptSet : my_point_sets)
            {
                // Find the circle center and its radius, given 3 points of the points set
                Circle circle;
                findCircle(circle, ptSet);
                
                // Check if the calculated circle is valid
                if (is_a_Circle(circle, ptSet))
                {
                    // Add the circle to our internal variable
                    my_circles.push_back(circle);
                } 
            }
        }

        // If the distance is greater than the radius, the point lies outside. 
        // If it's equal to the radius, the point lies on the circle
        bool is_a_Circle(Circle& aCircle, const PointSet& aPtSet)
        {            
            // Check if the cylindrical obstacle radius is greater then our threshold
            if (aCircle.radius > max_radius)
            {
                return false;
            }
            
            // Variable that stores our maximum threshold value for radius length
            const float ths = 0.01;
            for (int i=0; i < aPtSet.points.size(); i++)
            {
                // Calculate the distance between a point and the circle center
                float dist = length(aPtSet.points.at(i) - aCircle.center);
                // Check if the distance between a point and the circle center is equal to the radius of the circle (+= threshold)
                // If not this is not a cylindrical obstacle
                if (dist > (aCircle.radius + ths) ||
                    dist < (aCircle.radius -ths))
                {
                    return false;
                }
            }
            // It's a cylindrical obstacles
            return true;
        }

        // Function to calculate the center and the radius of a circle (cylindrical obstacle) given 3 Points
        void findCircle(Circle& circle, const PointSet& pt_set)
        {
            // We know that a set of point contain at least 3 points (min_group_points)
            cv::Point2f p1 = pt_set.first_point;
            cv::Point2f p2 = pt_set.middle_point;
            cv::Point2f p3 = pt_set.last_point;
            const float TOL = 0.0000001;

            // We calculate the perpendicular bisector of the segment between first point (x1,y1) and second point (x2,y2)
            // and the the perpendicular bisector of the segment between second point (x2,y2) and third point(x3,y3)
            // and we calculate the intersection of those lines, so that point will be the center
            // The radius is given by measuring the distance between the center and one of this point
            float yDelta_a = p2.y - p1.y; 
            float xDelta_a = p2.x - p1.x;
            float yDelta_b = p3.y - p2.y; 
            float xDelta_b = p3.x - p2.x;

            // Calculating m, which is the slope of the lines
            float aSlope = yDelta_a/xDelta_a;
            float bSlope = yDelta_b/xDelta_b;
            // Checking whether the given points are colinear.
            if (fabs(bSlope-aSlope) < TOL)
            {
                circle.center.x = 0.0;
                circle.center.y = 0.0;
                circle.radius = std::numeric_limits<float>::infinity();
                return;
            }
            circle.center.x = (aSlope * bSlope * (p1.y - p3.y) + bSlope * (p1.x + p2.x) - aSlope * (p2.x + p3.x) )/(2.0 *(bSlope-aSlope)); 
            circle.center.y = -1.0 * (circle.center.x - (p1.x + p2.x) / 2.0) / aSlope + (p1.y + p2.y) / 2.0;
            circle.radius = length(p2 - circle.center);
        }

        // Function that publish the detected cylindrical obstacles in the result vector
        void publishObstacles(const sensor_msgs::LaserScanConstPtr& aLaser_msg)
        {
            // We want the transform from frame base_laser_link to frame base_link
            // base_laser_link is the laser scanner position
            // base_link is the robot position
            // The laser scanner is mounted on front of the robot, +0.202m (along x axis) wrt to robot center
            tf::TransformListener listener;
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("base_link", aLaser_msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(10.0));
                listener.lookupTransform("base_link", aLaser_msg->header.frame_id.c_str(), ros::Time(0), transform);
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
                return;
            }

            for (const Circle& aCircle : my_circles)
            {
                geometry_msgs::Point base_laser_point;
                base_laser_point.x = aCircle.center.x;
                base_laser_point.y = aCircle.center.y;
                base_laser_point.z = 0.0;
                geometry_msgs::Point base_link_point = transformPoint(base_laser_point, transform);
                result_.obstacle.push_back(base_link_point);
            }

            // Clear the actual results, so we're ready for the next detection
            my_input_points.clear();
            my_point_sets.clear();
            my_circles.clear();
        }

        // Function that transform a point from a source frame (base_laser_link) to a target frame (base_link)
        geometry_msgs::Point transformPoint(const geometry_msgs::Point& point, const tf::StampedTransform& transform) 
        {
            geometry_msgs::Point tmp;
            tmp.x = point.x + transform.getOrigin().getX();
            tmp.y = point.y + transform.getOrigin().getY();
            tmp.z = 0.0;
            return tmp;
        }

        // Check that preempt has not been requested by the client
        bool preemptCB()
        {            
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                return true;
            }
            return false;
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

        // Function that store the message given by the laser scanner in some pointer
        bool updateLaser(sensor_msgs::LaserScanConstPtr& aLaser_msg)
        {
            aLaser_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh_, ros::Duration(1.0));
            if (aLaser_msg != NULL)
            {
                return true;
            }
            else
            {
                return false;
            }            
        }

        // Function that update the variables that stores the three distance (-90°), (90°) and (0°) given from the laser scanner
        void updateVariable(sensor_msgs::LaserScanConstPtr& aLaser_msg, int& idx_dx, int& idx_ct, int& idx_sx, float& angle_y_sx, float& angle_y_dx, float& y_sx, float& y_dx, float& x_ct)
        {
            // Find the three distance (-90°) and (90°)
            idx_dx = 60; // 90°
            idx_ct = aLaser_msg->ranges.size()/2; // 0°
            idx_sx = aLaser_msg->ranges.size() - 1 - idx_dx; // -90°
            
            angle_y_sx = aLaser_msg->angle_min + (aLaser_msg->angle_increment * idx_sx); // 90°
            angle_y_dx = aLaser_msg->angle_min + (aLaser_msg->angle_increment * idx_dx); // -90°

            y_sx = (aLaser_msg->ranges.at(idx_sx))*sin(angle_y_sx); // range at angle 90°
            y_dx = (aLaser_msg->ranges.at(idx_dx))*sin(angle_y_dx); // range at angle 
            x_ct = (aLaser_msg->ranges.at(idx_ct)); // range at angle 0°
        }

        // Function used to check if the robot is in the middle (+- threshold) wrt laser scan data
        bool checkDistance(const float& p1, const float& p2, float ths)
        {
            if (fabs(p1) <= fabs(p2) + ths &&
                fabs(p1) >= fabs(p2) - ths)
            {
                return true;
            }
            return false;
        }

        // Function that perform the extra point request. So we implement a "motion control law" for the robot.
        // The routine use the laser and send the velocity commands to the topic /mobile_base_controller/cmd_vel
        // to move the robot without calling the move_base stack
        void manual_moving_routine()
        {
            // This is necessary to maintain a constant speed as the mobile_base_controller only applies a given velocity for less than a second for safety reasons
            ros::Rate loop_r(5);

            // Variable that keep track of the status of the robot. If true the robot reach the end of the corridor
            bool stop = false;

            // Constant linear velocity of 0.5 m/s along the X axis
            float x_vel = 0.5;
            float z_rot = 0.05;

            // Variable that read the laser scan data
            sensor_msgs::LaserScanConstPtr laser_msg;
            
            // Variable used to set the linear velocity component and the angular velocity component of the robot
            geometry_msgs::Twist vel;

            // Set to 0 the useless parameter
            vel.linear.y = 0.0;
            vel.linear.z = 0.0;
            vel.angular.x = 0.0;
            vel.angular.y = 0.0;

            // Variable that stores the three distance (-90°), (90°) and (0°) and the  given from the laser scanner
            int idx_dx, idx_sx, idx_ct;
            float angle_y_sx, angle_y_dx, y_sx, y_dx, x_ct;
            
            
            // Keep iterating until we reach the end of the narrow corridor
            while(!stop)
            {
                // We will wait until we receive some pointer from the /scan topic
                laser_msg = NULL;                
                while (!updateLaser(laser_msg));
                // We update our internal variables
                updateVariable(laser_msg, idx_dx, idx_ct, idx_sx, angle_y_sx, angle_y_dx, y_sx, y_dx, x_ct);

                // First possible stop condition -> The robot see an obstacle in front of itself at less then 2m
                if(x_ct < 2.0)
                {
                    stop = true;
                    vel.linear.x = 0.0;
                    vel.angular.z = 0.0;
                }

                // Second possible stop condition -> The robot reach the end of the corridor
                else if (fabs(y_sx) > 2.0 || fabs(y_dx) > 2.0)
                {
                    stop = true;
                    vel.linear.x = 0.0;
                    vel.angular.z = 0.0;
                }

                // The robot is still inside the corridor
                else
                {
                    //If in the center go straight
                    if(checkDistance(y_sx, y_dx, 0.1))
                    {                        
                        vel.linear.x = x_vel;
                        // Leftwards rotation if positive value (rad/s)
                        vel.angular.z = 0.0;
                    }
                    // Rotate the robot and reach  the middle point
                    else
                    {
                        // The robot is too much on the right
                        if(fabs(y_sx) > fabs(y_dx))
                        {
                            // Stop the robot and correct the position
                            vel.linear.x = 0.0;
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Turn left by z_rot
                            vel.angular.z = z_rot;                            
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Stop the robot rotation
                            vel.angular.z = 0.0;
                            pub_vel.publish(vel);
                            loop_r.sleep();

                            // If we are in the middle wrt to laser scanner or an obstacle is detected in front of the robot (0.3m) stop this cycle
                            // Otherwise keep moving slowly (half of normal velocity) to reach the middle point
                            while(!checkDistance(y_sx, y_dx, 0.05) && x_ct > 0.3 && fabs(y_sx) > fabs(y_dx))
                            {
                                // If we are out of the corridor, stop this cycle
                                if (fabs(y_sx) > 2.0 || fabs(y_dx) > 2.0)
                                {
                                    break;
                                }
                                // Slow down and correct the position
                                vel.linear.x = x_vel*0.5;
                                pub_vel.publish(vel);
                                loop_r.sleep();

                                // Evaluate the position of the robot wrt laser scan
                                laser_msg = NULL;
                                while (!updateLaser(laser_msg));
                                // We update our internal variables
                                updateVariable(laser_msg, idx_dx, idx_ct, idx_sx, angle_y_sx, angle_y_dx, y_sx, y_dx, x_ct);
                            }
                            // Stop the robot and correct the position
                            vel.linear.x = 0.0;
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Turn right by z_rot, so we are sure to go straight
                            vel.angular.z = -z_rot;
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Stop the robot rotation                       
                            vel.angular.z = 0.0;
                        }
                        // The robot is too much on the left
                        else
                        {
                            // Stop the robot and correct the position
                            vel.linear.x = 0.0;
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Turn right by z_rot
                            vel.angular.z = -z_rot;                            
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Stop the robot rotation
                            vel.angular.z = 0.0;
                            pub_vel.publish(vel);
                            loop_r.sleep();


                            // If we are in the middle wrt to laser scanner or an obstacle is detected in front of the robot (0.3m) stop this cycle
                            // Otherwise keep moving slowly (half of normal velocity) to reach the middle point
                            while(!checkDistance(y_sx, y_dx, 0.05) && x_ct > 0.3 && fabs(y_dx) > fabs(y_sx))
                            {
                                // If we are out of the corridor, stop this cycle
                                if (fabs(y_sx) > 2.0 || fabs(y_dx) > 2.0)
                                {
                                    break;
                                }
                                // Slow down and correct the position
                                vel.linear.x = x_vel*0.5;
                                pub_vel.publish(vel);
                                loop_r.sleep();

                                // Evaluate the position of the robot wrt laser scan
                                laser_msg = NULL;
                                while (!updateLaser(laser_msg));
                                // We update our internal variables
                                updateVariable(laser_msg, idx_dx, idx_ct, idx_sx, angle_y_sx, angle_y_dx, y_sx, y_dx, x_ct);
                            }
                            // Stop the robot and correct the position                                                  
                            vel.linear.x = 0.0;
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Turn left by z_rot, so we are sure to go straight
                            vel.angular.z = z_rot;
                            pub_vel.publish(vel);
                            loop_r.sleep();
                            // Stop the robot rotation
                            vel.angular.z = 0.0;
                        }
                    }
                }

                // Publish the constant linear/angular velocity components to the topic /mobile_base_controller/cmd_vel
                pub_vel.publish(vel);
                // Do it every 0.2 second (1/5Hz)
                loop_r.sleep();
            }
        }

        // Function that send the Pose_B to the move_base stack 
        bool auto_moving_routine(const move_base_msgs::MoveBaseGoal& a_goal_pose)
        {
            // Create the action client
            // We also tell the action client that we want to spin a thread by default
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_("move_base", true);
            // Wait for the action server to come up
            while(!move_base_ac_.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            move_base_ac_.sendGoal(a_goal_pose);

            // Wait for the action to return
            move_base_ac_.waitForResult(); // Will wait for infinite time

            if(move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

    protected:
        ros::NodeHandle nh_;
        // Publisher in cmd_vel stack
        ros::Publisher pub_vel;
        // Tell the action client that we want to spin a thread by default
        actionlib::SimpleActionServer<assignment_1_group_27::TiagoAction> as_;
        std::string action_name_;
        assignment_1_group_27::TiagoFeedback feedback_;
        assignment_1_group_27::TiagoResult result_;
    public:
        TiagoAction(std::string name) : as_(nh_, name, boost::bind(&TiagoAction::executeCB, this, _1), false), action_name_(name)
        {
            as_.start();
            // mobile_base_controller/cmd_vel
            pub_vel = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
        }

        ~TiagoAction(void)
        {}

        // Define the executeCB(), function that will be called when the action server receives a goal
        void executeCB(const assignment_1_group_27::TiagoGoalConstPtr &goal)
        {
            // Helper variables
            ros::Rate r(1);
            // publish info to the console for the user
            ROS_INFO("%s: Executing", action_name_.c_str());
            result_.obstacle.clear();

            // The server just receive the input from the user (Pose_B)            
            feedback_.status = "The server has just receive the Pose_B";
            // Publish the feedback
            as_.publishFeedback(feedback_);
            r.sleep();

            // Check that preempt has not been requested by the client
            if (preemptCB())
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                //set the action state to aborted
                as_.setAborted(result_);
                return;
            }
            
            // EXTRA POINT 
            // Pointer to a robot pose message
            geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg = NULL;
            // We will wait until we receive some pointer from the /robot_pose topic
            while (pose_msg == NULL)
            { 
                pose_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", nh_, ros::Duration(5.0));
            }

            // Set the current status of the robot in the feedback message and publish it
            feedback_.status = "The robot is moving";
            as_.publishFeedback(feedback_);

            // If the robot is in the origin position then navigate through the narrow corridor using the laser data and
            // sending the velocity commands to the topic /mobile_base_controller/cmd_vel
            if (checkRobotPosition(pose_msg))
            {
                ROS_INFO("The robot is in the Starting Pose, using custom navigation to reach Pose_B");
                manual_moving_routine();
                ROS_INFO("Stopped custom navigation, now using move base stack");                
            }
            else
            {
                ROS_INFO("The robot isn't in the Starting Pose, using move base stack");
            }

            // We'll send a goal to the robot. The goal is to move from its actual position to Pose_B
            move_base_msgs::MoveBaseGoal goal_pose;
            goal_pose.target_pose.header.frame_id = "map";
            goal_pose.target_pose.header.stamp = ros::Time::now();
            goal_pose.target_pose.pose.position = goal->pose_B.position;
            goal_pose.target_pose.pose.orientation = goal->pose_B.orientation;  

            // Check if the robot reach the Pose_B
            if (auto_moving_routine(goal_pose))
            {
                ROS_INFO("The robot reach Pose_B");
                // Set the current status of the robot in the feedback message and publish it
                feedback_.status = "The robot reach Pose_B";
                as_.publishFeedback(feedback_);                
            }
            else
            {
                ROS_INFO("The robot failed to reach Pose_B");
                // Set the current status of the robot in the feedback message and publish it
                feedback_.status = "The robot failed to reach Pose_B";
                as_.publishFeedback(feedback_);
                as_.setAborted(result_);
                return;
            }

            // Check that preempt has not been requested by the client
            if (preemptCB())
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                //set the action state to aborted
                as_.setAborted(result_);
                return;
            }
            
            // Wait 4 seconds before reading the laser scan data
            // This time is necessary since the robot is stopped and the data given by the laser are consistent
            int i = 0;
            while (i < 4)
            {
                i++;
                r.sleep();           
            }

            ROS_INFO("The robot started the detection of the obstacles");
            // Set the current status of the robot in the feedback message and publish it
            feedback_.status = "The robot started the detection of the obstacles";
            as_.publishFeedback(feedback_);

            // Pointer to a laser scan message
            sensor_msgs::LaserScanConstPtr laser_msg = NULL;

            // We will wait until we receive some pointer from the /scan topic
            while (!updateLaser(laser_msg));
            
            // Analyze the Laser Scan data and we detect the cylindric obstacles
            obstacle_detection(laser_msg);          

            ROS_INFO("The detection is finished");
            // Set the current status of the robot in the feedback message and publish it
            feedback_.status = "The detection is finished";
            as_.publishFeedback(feedback_);
            
            // Populate the result message with the list of detected obstacles
            publishObstacles(laser_msg);
            ROS_INFO("Number of obstacle detected: %i", (int) result_.obstacle.size());
            
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // Set the result of the action and return it to the action client
            as_.setSucceeded(result_);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "server_tiago");
    TiagoAction tiago("server_tiago");
    ros::spin();
    return 0;
}
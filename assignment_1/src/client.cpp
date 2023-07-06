/**
 * Group 27
 * Luca Sambin
 * Andrea Fonsato
 * Zhaku Bejaj
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <assignment_1_group_27/TiagoAction.h>

// Called once when the goal completes -> At the end
void doneCb(const actionlib::SimpleClientGoalState& state,
            const assignment_1_group_27::TiagoResultConstPtr& result)
{
    // Print the number of obstacle detected
    ROS_INFO("Number of obstacle detected: %i", (int) result->obstacle.size());
    for(int i=0; i < result->obstacle.size(); i++)
    {
        // Print for each obstacle, its position
        ROS_INFO("Position of obstacle %i: [x: %f, y: %f]", i+1, result->obstacle.at(i).x, result->obstacle.at(i).y);
    }
}

// Called once when the goal becomes active -> At the beginning
// When executeCB() on server start
void activeCb() {}

// Called every time feedback is received for the goal
void feedbackCb(const assignment_1_group_27::TiagoFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback, Current Tiago status: %s", feedback->status.c_str());
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "client_tiago");

    // Check the number of parameters. Position[x,y,z] and Orientation[x,y,z,w]
    if (argc != 8)
    {
        ROS_INFO("Pose_B is not correct");
        ROS_INFO("Wrong number of parameter: %i. It should be 7!", argc);
        ROS_INFO("Position [x,y,z] and Orientation [x,y,z,w]");
        return -1;
    }

    // Create the action client. We also tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<assignment_1_group_27::TiagoAction> ac("server_tiago", true);
    ROS_INFO("Waiting for action server to start");
    // Wait for the action server to come up
    ac.waitForServer(); // Will wait for infinite time
    ROS_INFO("Action server started, sending goal");

    // Read the user input pose (the goal Pose_B) from the command line
    // Convert a string to float
    assignment_1_group_27::TiagoGoal goal;
    goal.pose_B.position.x = atof(argv[1]);
    goal.pose_B.position.y = atof(argv[2]);
    goal.pose_B.position.z = atof(argv[3]);
    goal.pose_B.orientation.x = atof(argv[4]);
    goal.pose_B.orientation.y = atof(argv[5]);
    goal.pose_B.orientation.z = atof(argv[6]);
    goal.pose_B.orientation.w = atof(argv[7]);

    // Send a goal to the action server
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    
    // Wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));
    
    // If the goal finished before the time out the goal status is reported
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action Server finish: %s",state.toString().c_str());
    }
    // Otherwise the user is notified that the goal did not finish in the allotted time.
    else
    {
        ROS_INFO("Action Server did not finish before the time out");
    }
    
    return 0;
}
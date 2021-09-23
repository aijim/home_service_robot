#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "pick_objects");

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    float pick_up_x, pick_up_y, pick_up_w, drop_off_x, drop_off_y, drop_off_w;
    ros::NodeHandle n;
    std::string node_name = ros::this_node::getName();
    n.getParam(node_name+"/pick_up_x", pick_up_x);
    n.getParam(node_name+"/pick_up_y", pick_up_y);
    n.getParam(node_name+"/pick_up_w", pick_up_w);
    n.getParam(node_name+"/drop_off_x", drop_off_x);
    n.getParam(node_name+"/drop_off_y", drop_off_y);
    n.getParam(node_name+"/drop_off_w", drop_off_w);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pick_up_x;
    goal.target_pose.pose.position.y = pick_up_y;
    goal.target_pose.pose.orientation.w = pick_up_w;

    ROS_INFO("Sending goals");
    ac.sendGoal(goal);
    
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Moved to pick up position");
    }else{
        if(ac.getState() == actionlib::SimpleClientGoalState::PENDING)
          ROS_INFO("PENDING");
        else if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
          ROS_INFO("ACTIVE");
         else if(ac.getState() == actionlib::SimpleClientGoalState::RECALLED)
          ROS_INFO("RECALLED");
        else if(ac.getState() == actionlib::SimpleClientGoalState::REJECTED)
          ROS_INFO("REJECTED");
         else if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
          ROS_INFO("PREEMPTED");
         else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
          ROS_INFO("ABORTED");
         else if(ac.getState() == actionlib::SimpleClientGoalState::LOST)
          ROS_INFO("LOST");
        ROS_INFO("Failed to move to pick up position");
    }

    ros::Duration(5).sleep();

    goal.target_pose.pose.position.x = drop_off_x;
    goal.target_pose.pose.position.y = drop_off_y;
    goal.target_pose.pose.orientation.w = drop_off_w;

    ROS_INFO("Sending goals");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Moved to final position");
    }else{
        ROS_INFO("Failed to move to final position");
    }

    return 0;
}

    

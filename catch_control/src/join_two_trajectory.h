//
// Created by yang on 5/7/19.
//

#ifndef CATCH_CONTROL_JOIN_TWO_TRAJECTORY_H
#define CATCH_CONTROL_JOIN_TWO_TRAJECTORY_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <visualization_msgs/MarkerArray.h>
#include <kdl/frames.hpp>

#include <turtlesim/Pose.h>
#include <tf/transform_broadcaster.h>

class Join_trajectory {

public:
    Join_trajectory(ros::NodeHandle _nh, std::string _PLANNING_GROUP, double marker_bias_threshold = 0.05);

    ~Join_trajectory(){delete client_;}

    void subMotionCaptureCB(visualization_msgs::MarkerArray state);

    void keyboard_callback(turtlesim::PosePtr msg);

    moveit_msgs::RobotTrajectory &join_tra(moveit_msgs::RobotTrajectory &tra_fir, moveit_msgs::RobotTrajectory &tra_sec,
                                           std::vector<double> cur_joint_value);

    bool link_two_tra(std::vector<trajectory_msgs::JointTrajectoryPoint>& points,
                      double plan_time, std::vector<std::string> joint_name, geometry_msgs::Pose pose,
                      bool pose_or_joint = true, std::vector<double> *joint_value_C=0);



private:

    ros::NodeHandle nh_;
    std::string PLANNING_GROUP;
    ros::Subscriber sub_markers_;
    ros::Subscriber sub_turtle_;
    ros::Publisher pub_hand_cmd_;
    bool received_first_marker = false;
    double marker_threshold;
    tf::TransformBroadcaster bro_trans;

public:
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
    Client *client_;
    geometry_msgs::Point marker_position;
    geometry_msgs::Point old_marker_position = marker_position;
    bool over_bias_threshold = false;
    moveit::planning_interface::MoveGroupInterface move_group;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    control_msgs::FollowJointTrajectoryGoal my_goal;

};

Join_trajectory::Join_trajectory(ros::NodeHandle _nh, std::string _PLANNING_GROUP, double marker_bias_threshold)
                                 : nh_(_nh), PLANNING_GROUP(_PLANNING_GROUP), marker_threshold(marker_bias_threshold),
                                 move_group(_PLANNING_GROUP)
                                 {

    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    sub_markers_ = nh_.subscribe("/vis_markers", 1, &Join_trajectory::subMotionCaptureCB,this);
    sub_turtle_ = nh_.subscribe("/turtle1/pose", 1, &Join_trajectory::keyboard_callback,this);
    client_= new Client("/UR5e/follow_joint_trajectory", true);
    client_->waitForServer(ros::Duration());

}

void Join_trajectory::keyboard_callback(turtlesim::PosePtr msg) {

    tf::Transform tra_bro;
    tra_bro.setOrigin(tf::Vector3((msg->x-5.5)/5.5, 0.9, (msg->y)/5.5));
    marker_position.x = (msg->x - 5.5)/5.5;
    marker_position.y = 0.9;
    marker_position.z = (msg->y)/5.5;
    tf::Quaternion quat;
    quat.setRPY(0,-msg->theta,0);
    tra_bro.setRotation(quat);
    bro_trans.sendTransform(tf::StampedTransform(tra_bro,ros::Time::now(),"ur_world","turtle_trans"));
}

//  连接两条轨迹
bool Join_trajectory::link_two_tra(std::vector<trajectory_msgs::JointTrajectoryPoint>& points,
        double plan_time_leap, std::vector<std::string> joint_name, geometry_msgs::Pose pose,
        bool pose_or_joint, std::vector<double> *joint_value_C) {

    ros::Time start_time = ros::Time::now();
    for(unsigned int i=0;i<points.size();i++)
    {
        if(ros::Duration(plan_time_leap) < points[i].time_from_start)
        {
            std::vector<double> start_joint_value = points[i].positions;

            robot_state::RobotStatePtr start_state = move_group.getCurrentState();
            start_state->setJointGroupPositions(joint_model_group,start_joint_value);
            move_group.setStartState(*start_state);
            if(pose_or_joint)
                move_group.setPoseTarget(pose);
            else
                move_group.setJointValueTarget(*joint_value_C);

            move_group.plan(my_plan);
            my_goal.trajectory.points = my_plan.trajectory_.joint_trajectory.points;
            my_goal.trajectory.header.stamp = ros::Time::now();
            my_goal.trajectory.joint_names = joint_name;
            while (ros::Time::now() - start_time < ros::Duration(plan_time_leap)) {

            }
            client_->sendGoal(my_goal);

            break;
        }
    }
    return true;
}



//  订阅动捕回调
void Join_trajectory::subMotionCaptureCB(visualization_msgs::MarkerArray state){
    static int false_number = 0;

    try {
        marker_position = state.markers.at(2).pose.position;
        if(!received_first_marker)
            old_marker_position = marker_position;
        double marker_bias_pow2 = pow(marker_position.x-old_marker_position.x,2) +
                pow(marker_position.y-old_marker_position.y,2) + pow(marker_position.z-old_marker_position.z,2);
//        marker_bias = sqrt(marker_bias_pow2);
//        ROS_INFO_STREAM(marker_bias_pow2<<" "<<marker_bias);
        if(sqrt(marker_bias_pow2) > marker_threshold && !over_bias_threshold )
        {
            over_bias_threshold = true;
            old_marker_position = marker_position;
        }
        received_first_marker = true;
    }
    catch (std::out_of_range){
        if(!(false_number % 100))
            std::cout<<"cann't find the marker";
        false_number++;
    }


}

moveit_msgs::RobotTrajectory& Join_trajectory::join_tra(moveit_msgs::RobotTrajectory &tra_fir, moveit_msgs::RobotTrajectory &tra_sec,
                               std::vector<double> cur_joint_value) {
    double min_point_distance = 0;
    unsigned int point_number = tra_sec.joint_trajectory.points.size();
    unsigned int i = 0; //想要的点序号
    for (i=0;i<point_number;i++)
    {
        double distance_mid = 0;
        for(unsigned j=0;j<cur_joint_value.size();j++)
        {
            distance_mid += pow(tra_sec.joint_trajectory.points[i].positions[j]-cur_joint_value[j],2);
        }
        if(sqrt(distance_mid) < min_point_distance)
            min_point_distance = sqrt(distance_mid);
    }
//    tra_sec.joint_trajectory.points.;  //确认一下改时间是否有效
    return tra_sec;
}


#endif //CATCH_CONTROL_JOIN_TWO_TRAJECTORY_H

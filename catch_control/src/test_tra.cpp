//
// Created by yang on 3/31/19.  ur_kinematics/UR5KinematicsPlugin
//
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/CollisionObject.h>
#include "mymoveit.h"
#include "join_two_trajectory.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <visualization_msgs/MarkerArray.h>



int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_tra_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();


    bool imediately;
    double bias_marker_x, velocity_factor, plan_rate, marker_bias_threshold, plan_time_leap;
    nh.param("bias_marker_x",bias_marker_x,0.0);
    nh.param("velocity_factor",velocity_factor,0.5);
    nh.param("plan_rate",plan_rate,1.0);
    nh.param("marker_bias_threshold",marker_bias_threshold,0.0);
    nh.param("imediately",imediately,true);
    nh.param("plan_time_leap",plan_time_leap,0.5);
    ROS_INFO_STREAM("bias_marker_x "<<bias_marker_x<<
                    "\tvelocity_factor "<<velocity_factor<<
                    "\tplan_rate "<<plan_rate<<
                    "\timediately "<<imediately<<
                    "\tplan_time_leap "<<plan_time_leap);

    KDL::Frame base2ur_world;
    base2ur_world.p = KDL::Vector(-0.01 ,0.377 ,0.808);
    base2ur_world.M = KDL::Rotation::RPY(0,0,1.570796327);


//    display in rviz
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("ur_world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    std::string PLANNING_GROUP = "manipulator";
    Join_trajectory my_join(nh,PLANNING_GROUP);


    my_join.move_group.allowReplanning(true);
    my_join.move_group.setMaxVelocityScalingFactor(velocity_factor);   //设置速度上限因子

    ROS_INFO_STREAM("planning frame: "<< my_join.move_group.getPlanningFrame(); );
    std::vector<std::string> joint_name = my_join.move_group.getJointNames();

    sleep(1);   //需要睡眠一会

    double arr_joint_A[] = {1,-1.3,-2.1,-1.1,-1.6,-1.8};
    double arr_joint_B[] = {-1,-1.3,-2.1,-1.1,-1.6,-1.8};
    double arr_joint_C[] = {-2,-0.5,-2.1,-1.1,-1.6,-1.8};
    double start_joint[] = {-2.52, -1.66, -1.37, 3.03, -0.95, 1.57};
    std::vector<double> joint_value_A(arr_joint_A,arr_joint_A + 6);
    std::vector<double> joint_value_B(arr_joint_B,arr_joint_B + 6);
    std::vector<double> joint_value_C(arr_joint_C,arr_joint_C + 6);
    std::vector<double> joint_value_start(start_joint,start_joint + 6);

    my_join.move_group.setStartStateToCurrentState();
    my_join.move_group.setJointValueTarget(joint_value_start);
    my_join.move_group.move();
//
//
//    my_join.move_group.setJointValueTarget(joint_value_B);
//    my_join.move_group.plan(my_join.my_plan);
//    my_join.my_goal.trajectory.points = my_join.my_plan.trajectory_.joint_trajectory.points;
//    my_join.my_goal.trajectory.joint_names = joint_name;
//    my_join.my_goal.trajectory.header.stamp = ros::Time::now();
//
//    visual_tools.publishTrajectoryLine(my_join.my_plan.trajectory_, my_join.joint_model_group);
//    visual_tools.trigger();
//
//    my_join.client_->sendGoal(my_join.my_goal);
    ros::Time start_time = ros::Time::now();
    ROS_INFO_STREAM("start_time "<<start_time);


//    my_join.link_two_tra(my_join.my_plan.trajectory_.joint_trajectory.points, plan_time_leap,
//                         joint_name,geometry_msgs::Pose(),false,&joint_value_C);
//
//    visual_tools.publishTrajectoryLine(my_join.my_plan.trajectory_, my_join.joint_model_group,rviz_visual_tools::YELLOW);
//    visual_tools.trigger();


    tf::Quaternion bottle_qua;
    bottle_qua.setRPY(-M_PI_2,-M_PI_2,0);  //M_PI_2,0,M_PI_2

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x=bottle_qua.x();
    target_pose.orientation.y=bottle_qua.y();
    target_pose.orientation.z=bottle_qua.z();
    target_pose.orientation.w=bottle_qua.w();

    double distance = 1;
//    ros::Time plan_start_time = ros::Time::now();
    while (ros::ok())
    {
//        ros::Rate loop_rate(10);
//        ROS_INFO_STREAM("distance "<<distance);

        KDL::Vector marker2map(my_join.marker_position.x,my_join.marker_position.y,my_join.marker_position.z);
        KDL::Vector marker2ur_world = marker2map;          //turtle1 坐标 未转换
//        ROS_INFO_STREAM("marker position in ur_world  "<<marker2ur_world.x()<<" "<<marker2ur_world.y()<<" "<<marker2ur_world.z());
        target_pose.position.x = marker2ur_world.x();
        target_pose.position.y = marker2ur_world.y();
        target_pose.position.z = marker2ur_world.z();
        my_join.move_group.setPoseTarget(target_pose);

        if(distance > 0.005)
        {
            bool success = (my_join.move_group.plan(my_join.my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//            ros::Time plan_end_time = ros::Time::now();
            if(success)
            {
                my_join.link_two_tra(my_join.my_plan.trajectory_.joint_trajectory.points, plan_time_leap,
                                     joint_name,target_pose);
//                visual_tools.publishTrajectoryLine(my_join.my_plan.trajectory_, my_join.joint_model_group);
//                visual_tools.trigger();
            }
        }

        geometry_msgs::Point point_now = my_join.move_group.getCurrentPose().pose.position;

        double distance_pow2 =  pow((point_now.x-target_pose.position.x),2) +
                pow((point_now.y-target_pose.position.y),2) + pow((point_now.z-target_pose.position.z),2);
        distance = sqrt(distance_pow2);

//        loop_rate.sleep();
    }

//    test join two tra
//    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//    const robot_state::JointModelGroup* joint_model_group =
//            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan_sec;
//    control_msgs::FollowJointTrajectoryGoal my_goal;
//    start_time = ros::Time::now();
//
//    for(unsigned int i=0;i<my_join.my_plan.trajectory_.joint_trajectory.points.size();i++)
//    {
//
//        if(ros::Duration(plan_time_leap) < my_join.my_plan.trajectory_.joint_trajectory.points[i].time_from_start)
//        {
//            std::vector<double> start_joint_value = my_join.my_plan.trajectory_.joint_trajectory.points[i].positions;
//
//            robot_state::RobotStatePtr start_state = move_group.getCurrentState();
//            start_state->setJointGroupPositions(joint_model_group,start_joint_value);
//            move_group.setStartState(*start_state);
//            move_group.setJointValueTarget(joint_value_C);
//            move_group.plan(my_plan);
//            my_goal.trajectory.points = my_plan.trajectory_.joint_trajectory.points;
//            my_goal.trajectory.header.stamp = ros::Time::now();
//            my_goal.trajectory.joint_names = joint_name;
//            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group,rviz_visual_tools::YELLOW);
//            visual_tools.trigger();
//            while (ros::Time::now() - start_time < ros::Duration(plan_time_leap)) {
//
//            }
//
//            my_join.client_->sendGoal(my_goal);
//            break;
//        }
//    }

    ROS_INFO_STREAM("plan next time "<<ros::Time::now() - start_time);


    ros::waitForShutdown();
}
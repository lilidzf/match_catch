//
// Created by yang on 3/31/19.  ur_kinematics/UR5KinematicsPlugin
//
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/CollisionObject.h>
//#include "mymoveit.h"
#include "join_two_trajectory.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <visualization_msgs/MarkerArray.h>




int main(int argc,char **argv)
{
    ros::init(argc,argv,"demo_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    double bias_marker_x, velocity_factor, plan_rate, marker_bias_threshold;
    nh.param("bias_marker_x",bias_marker_x,0.0);
    nh.param("velocity_factor",velocity_factor,0.5);
    nh.param("plan_rate",plan_rate,1.0);
    nh.param("marker_bias_threshold",marker_bias_threshold,0.0);
    ROS_INFO_STREAM("bias_marker_x "<<bias_marker_x<<
                    "\tvelocity_factor "<<velocity_factor<<
                    "\tplan_rate "<<plan_rate);

    typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;
    Client *client_;
    client_= new Client("/UR5e/follow_joint_trajectory", true);
    client_->waitForServer(ros::Duration());

//    display in rviz
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("ur_world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

//    UR_gripper ur5e_gripper(nh,"manipulator");
//    ur5e_gripper.robotiq_hand_move(85,5,50);

    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.allowReplanning(true);
    move_group.setMaxVelocityScalingFactor(velocity_factor);   //设置速度上限因子
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO_STREAM("planning frame: "<< move_group.getPlanningFrame(); );


    Join_trajectory my_join(nh,PLANNING_GROUP);
    sleep(1);   //需要睡眠一会

//    transform from base_link to map
    KDL::Frame base_link2map;
    base_link2map.p = KDL::Vector(0.310028, -0.412067 ,0.0397502);
    base_link2map.M = KDL::Rotation::Quaternion(0.000413926 ,-0.000457995 ,-0.704761 ,0.709444);

    KDL::Frame base2ur_world;
    base2ur_world.p = KDL::Vector(-0.01 ,0.377 ,0.808);
    base2ur_world.M = KDL::Rotation::RPY(0,0,1.570796327);

    KDL::Frame map2ur_world = base2ur_world * base_link2map.Inverse();


    tf::Quaternion bottle_qua;
    bottle_qua.setRPY(M_PI,0,0);  //M_PI_2,0,M_PI_2

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x=bottle_qua.x();
    target_pose.orientation.y=bottle_qua.y();
    target_pose.orientation.z=bottle_qua.z();
    target_pose.orientation.w=bottle_qua.w();
    double distance = 1.0;

    bool first_plan = true;  //flag for the first plan

    //    move_group.asyncExecute(my_plan);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_first;
    control_msgs::FollowJointTrajectoryGoal my_goal;
    control_msgs::FollowJointTrajectoryActionFeedback last_feedback;
    my_goal.trajectory.joint_names = move_group.getJointNames();

    ros::Time plan_start_time = ros::Time::now();
    while (ros::ok())
    {
//        ros::Rate loop_rate(plan_rate);
        ROS_INFO_STREAM("over_bias_threshold "<<my_join.over_bias_threshold);
        ROS_INFO_STREAM("distance "<<distance);

        KDL::Vector marker2map(my_join.marker_position.x,my_join.marker_position.y,my_join.marker_position.z);
        KDL::Vector marker2ur_world = map2ur_world * marker2map;
//        KDL::Vector marker2ur_world = marker2map;          //turtle1 坐标 未转换
//            ROS_INFO_STREAM("marker position in map  "<<marker2map[0]<<" "<<marker2map[1]<<" "<<marker2map[2]);
//            ROS_INFO_STREAM("marker position in ur_world  "<<marker2ur_world.x()<<" "<<marker2ur_world.y()<<" "<<marker2ur_world.z());
        target_pose.position.x = marker2ur_world.x() - bias_marker_x;
        target_pose.position.y = marker2ur_world.y();
        target_pose.position.z = marker2ur_world.z() + 0.3;
        move_group.setPoseTarget(target_pose);

        if(distance > 0.005)
//            if(distance > 0.005 && my_join.over_bias_threshold == true)
        {
            my_join.over_bias_threshold = false;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ros::Time plan_end_time = ros::Time::now();
            if(success)
            {
                visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
                visual_tools.trigger();
                if(first_plan) {
                    plan_start_time = plan_end_time;
                    first_plan = false;
                    my_plan_first = my_plan;
                }
                std::vector<double> cur_joint_value = move_group.getCurrentJointValues();
                my_goal.trajectory.header.stamp = ros::Time::now() - (plan_end_time-plan_start_time);
                ROS_INFO_STREAM("plan+sleep circle time "<< plan_end_time-plan_start_time);
                std::vector<double> cur_joint_value_feed = last_feedback.feedback.actual.positions;
                my_goal.trajectory.points = my_plan.trajectory_.joint_trajectory.points;
                client_->sendGoal(my_goal);   //即使规划不成功造成两次结果一样也没关系
                plan_start_time = ros::Time::now();
            }
        }

        geometry_msgs::Point point_now = move_group.getCurrentPose().pose.position;

//        ROS_INFO_STREAM("point_now "<<point_now.x<<" "<<point_now.y<<" "<<point_now.z);
        double distance_pow2 =  pow((point_now.x-target_pose.position.x),2) +
                pow((point_now.y-target_pose.position.y),2) + pow((point_now.z-target_pose.position.z),2);
        distance = sqrt(distance_pow2);
//        loop_rate.sleep();
    }


    delete client_;

    ros::waitForShutdown();
}
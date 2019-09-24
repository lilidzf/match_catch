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




int main(int argc,char **argv)
{
    ros::init(argc,argv,"mymoveit_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();


    UR_gripper ur5e_gripper(nh,"manipulator");
    ur5e_gripper.robotiq_hand_move(85,5,50);


    double bottle_height , gripper_z;
    std::vector<double> bottle_x_y_z;


    nh.param("bottle_height", bottle_height, 0.06);
    nh.param("bottle_x_y_z", bottle_x_y_z, {0.2, 0, 0.793});
    nh.param("gripper_z", gripper_z, 1.0);

    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> joint_name;
    joint_name = move_group.getJointNames();


    ROS_INFO_STREAM("planning frame: "<< move_group.getPlanningFrame(); );


//    move_group.setNamedTarget("usual");
//    move_group.move();
//    sleep(2);

    //    发布物体
    moveit_msgs::CollisionObject collision_object;
    ur5e_gripper.add_box(collision_object,bottle_height,bottle_x_y_z,"box1","ur_world");


//  轨迹重定义
//    moveit::planning_interface::MoveGroupInterface::Plan plan2;
//    bool success2 = move_group.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
//    ur5e_gripper.scale_trajectory_speed(plan2, 0.1);
//    move_group.execute(plan2);


    geometry_msgs::Pose target_pose1;
    tf::Quaternion bottle_qua;
    bottle_qua.setRPY(M_PI,0,0);
//    ROS_INFO_STREAM(bottle_qua.x()<<bottle_qua.y()<<bottle_qua.z()<<bottle_qua.w());
    target_pose1.orientation.x=bottle_qua.x();
    target_pose1.orientation.y=bottle_qua.y();
    target_pose1.orientation.z=bottle_qua.z();
    target_pose1.orientation.w=bottle_qua.w();
    target_pose1.position.x = bottle_x_y_z[0];
    target_pose1.position.y = bottle_x_y_z[1];

//    target_pose1.position.z = bottle_x_y_z[2]+bottle_height/2;
    target_pose1.position.z = gripper_z;

    move_group.allowReplanning(true);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);

    move_group.setPoseTarget(target_pose1);
    ROS_INFO("move to target pose");

    move_group.setPlanningTime(10);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success3 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    int tries = 1;
    while (!success3 && tries <10)
    {
        ROS_INFO("This is  %d th try",tries);
        success3 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        tries++;
    }

    move_group.move();
    sleep(3);
    ur5e_gripper.robotiq_hand_move(56,5,50);
    ros::Duration(3).sleep();


//    物体附在机器人上
    move_group.attachObject(collision_object.id);

//    搬运物体
    ur5e_gripper.plan_CartesianPath();
    sleep(2);

    ur5e_gripper.robotiq_hand_move(85,5,50);

//    移除障碍物
    move_group.detachObject(collision_object.id);
    ros::Duration(3).sleep();

//    移动到另一个位置
    geometry_msgs::Pose target_pose2 = move_group.getCurrentPose().pose;
    target_pose2.position.z += 0.1;
    move_group.setPoseTarget(target_pose2);
    move_group.move();

//    删除障碍物
    ros::Duration(3).sleep();
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    ros::shutdown();
//    ros::waitForShutdown();
}
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
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"demo_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("ur_world");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();


    UR_gripper ur5e_gripper(nh,"manipulator");
    ur5e_gripper.robotiq_hand_move(85,5,50);


    double bottle_height , gripper_z;
    std::vector<double> bottle_x_y_z;


    nh.param("bottle_height", bottle_height, 0.06);
    nh.param("bottle_x_y_z", bottle_x_y_z, {0.2, 0, 0.868});
    nh.param("gripper_z", gripper_z, 1.072);

    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    ROS_INFO_STREAM("planning frame: "<< move_group.getPlanningFrame(); );

//    长方体障碍物顶点坐标 已知长宽
    KDL::Vector myvec_box(0.27469,0.1837,0.633);

    KDL::Vector myvec_box_final;   //障碍物
    KDL::Vector myvec_box2_final;  //魔方

//    障碍物相对位置
    KDL::Frame base_link2map,new_frame;
    base_link2map.p = KDL::Vector(0.310028, -0.412067 ,0.0397502);
    base_link2map.M = KDL::Rotation::Quaternion(0.000413926 ,-0.000457995 ,-0.704761 ,0.709444);
//    计算逆矩阵
    new_frame = base_link2map.Inverse();
    double x,y,z,w;
    new_frame.M.GetQuaternion(x,y,z,w);
    ROS_INFO_STREAM("base_link2map "<<new_frame.p.data[0]<<" "<<new_frame.p.data[1]<<" "<<new_frame.p.data[2]<<" "<<x<<" "<<y<<" "<<z<<" "<<w<<std::endl);

    myvec_box_final = base_link2map.Inverse() * myvec_box;

    KDL::Frame base2ur_world;
    base2ur_world.p = KDL::Vector(-0.01 ,0.377 ,0.808);
    base2ur_world.M = KDL::Rotation::RPY(0,0,1.570796327);


//    魔方相对障碍物位置
    bottle_x_y_z[0] = myvec_box_final[0] ;
    bottle_x_y_z[1] = myvec_box_final[1] + 0.32;
    bottle_x_y_z[2] = 0.1;

//    魔方相对 ur_world 位置
    KDL::Vector myvec_box2_begin(bottle_x_y_z[0],bottle_x_y_z[1],bottle_x_y_z[2]);
    myvec_box2_final = base2ur_world * myvec_box2_begin;

    //    发布物体
    moveit_msgs::CollisionObject collision_object;
    moveit_msgs::CollisionObject collision_object2;
    ur5e_gripper.add_box(collision_object,bottle_height,bottle_x_y_z,"box1","base_link");


    collision_object2.header.frame_id = "base_link";
    collision_object2.id = "box2";



    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = bottle_height;
    primitive.dimensions[1] = bottle_height;
    primitive.dimensions[2] = myvec_box_final[2];

    geometry_msgs::Pose bottle_pose;
    bottle_pose.orientation.w = 1.0;
    bottle_pose.position.x = myvec_box_final[0];
    bottle_pose.position.y = myvec_box_final[1];
    bottle_pose.position.z = myvec_box_final[2]/2;

    collision_object2.primitives.push_back(primitive);
    collision_object2.primitive_poses.push_back(bottle_pose);
    collision_object2.operation = collision_object2.ADD;

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.applyCollisionObject(collision_object2);


    geometry_msgs::Pose target_pose1;
    tf::Quaternion bottle_qua;
    bottle_qua.setRPY(M_PI,0,0);
//    ROS_INFO_STREAM(bottle_qua.x()<<bottle_qua.y()<<bottle_qua.z()<<bottle_qua.w());
    target_pose1.orientation.x=bottle_qua.x();
    target_pose1.orientation.y=bottle_qua.y();
    target_pose1.orientation.z=bottle_qua.z();
    target_pose1.orientation.w=bottle_qua.w();
    target_pose1.position.x = myvec_box2_final[0];
    target_pose1.position.y = myvec_box2_final[1];

//    target_pose1.position.z = bottle_x_y_z[2]+bottle_height/2;
    target_pose1.position.z = gripper_z ;

    move_group.allowReplanning(true);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setMaxVelocityScalingFactor(0.05);   //设置速度上限因子
//    move_group.setMaxAccelerationScalingFactor(0.1);

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

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group.execute(my_plan);
    sleep(3);
    ur5e_gripper.robotiq_hand_move(56,5,50);
    ros::Duration(3).sleep();


    ros::shutdown();
}
//
// Created by zd on 19-6-1.  基于动作捕捉系统的人机交互简单demo  研电赛
//

#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "human_interact_demo.h"


int main(int argc,char **argv)
{
    ros::init(argc,argv,"human_interact_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    bool pose_orien = false;
    nh.param("pose_orien", pose_orien, false);
//    std::vector<double> rgb_bias_x_y;
//    nh.param("rgb_bias_x_y", rgb_bias_x_y, {0.0, 0.0, 0.0});

    std::string PLANNING_GROUP = "manipulator";
    UR_gripper ur5e_gripper(nh,PLANNING_GROUP);

    double joint_satrt_arr[] = { 1.15, -1.82, -1.81, -1.09, 1.56, -0.91 };
//    std::vector<double> start_joint(joint_satrt_arr, joint_satrt_arr+6);
//    ur5e_gripper.move_to_start_position(start_joint);
//    ur5e_gripper.robotiq_hand_move(85,5,50);
    KDL::JntArray start_joint(6);
    for(int i=0;i<6;i++)
    {
        start_joint(i) = joint_satrt_arr[1];
    }
    ur5e_gripper.servoj_moveto(start_joint,3,true);
//    选择水果
    Eigen::Vector3d choosed_fruit(-0.309, 0.028, 1.01);
//    do{
//        choosed_fruit = ur5e_gripper.choose_fruit();
//    }while(choosed_fruit[0] = 0.0);
    std::vector<double> fruit_pos;
    fruit_pos.push_back(choosed_fruit[0]);
    fruit_pos.push_back(choosed_fruit[1]);
    fruit_pos.push_back(choosed_fruit[2]);

//    ur5e_gripper.catch_rgb_fruit(fruit_pos);
    KDL::Vector fruit_pos_vec(fruit_pos[0], fruit_pos[1], fruit_pos[2]);
    ur5e_gripper.test_move_line(fruit_pos_vec);


//    ros::Duration(2).sleep();
//
//    if(pose_orien)
//        ur5e_gripper.catch_pose_box(2);
//    else
//        ur5e_gripper.catch_rgb_fruit(2);


    //    发布物体
//    double bottle_size[] = { 0.116 , 0.033};   //易拉罐高 半径
//    moveit_msgs::CollisionObject collision_object;
//    ur5e_gripper.add_box(collision_object,bottle_size,bottle_center,grasp_frame_urWorld, "box1","ur_world");

    ros::waitForShutdown();
    return 1;
}
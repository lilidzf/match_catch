#include <ros/ros.h>
#include "catch_bottle.h"
#include <vector>
#include <istream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "catch_control");
    ros::NodeHandle nh;

    std::string chain_start, chain_end, urdf_param;
    double timeout;
    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("UR5e/robot_description"));
    std::string UR_name_prefix,hand_name_prefix;
    nh.getParam("UR_name_prefix", UR_name_prefix);
    nh.getParam("hand_name_prefix", hand_name_prefix);
    ROS_INFO_STREAM("UR_name_prefix: "<<UR_name_prefix<<"   hand_name_prefix:"<<hand_name_prefix );

    UR UR5e(nh, urdf_param, "base_link", "tool0", timeout, UR_name_prefix,hand_name_prefix);
    ros::Duration(0.5).sleep();
    ros::spinOnce();
//    KDL::JntArray tmp(6);
//    std::vector<double> gravity_up_joints;
//    nh.getParam("gravity_up_joints", gravity_up_joints);
//    for (int j = 0; j <6; ++j) {
//        tmp(j) = gravity_up_joints[j]/180*M_PI;
//    }

//    double angle = 30;
    ROS_INFO("Wait for movement");

    UR5e.GetCurr();
    for(int i =1 ; i<100 ;i++) {
        UR5e.move_to_xyz(0, 0.001, 0, 0.1, 1);
    }
//    UR5e.move_straight(0.144686,0.436581,0.802392,5);

//    UR5e.hand_ExtStop();
//    UR5e.SetMaxVelocity(16000);
//    ros::Duration(2).sleep();
//    UR5e.lei_hand_move(16000,1);

//    ros::Duration(2).sleep();
////
//    UR5e.hand_init();
//////////
//    UR5e.HandMoveToFixedPosition(20);
////////
//    UR5e.HandMoveToFixedPosition(30);
//////
//    UR5e.HandMoveToFixedPosition(90);
////////
////    UR5e.HandMoveToFixedPosition(50);
//////
//    UR5e.HandMoveToFixedPosition(10);
////
//    UR5e.HandMoveToFixedPosition(0);
////




//    double ans = UR5e.TransAngleToDisplace(angle);
//    std::cout << "ans: " << ans << std::endl;


//    ros::Duration(2).sleep();
////    UR5e.hand_move_init(0);
//    UR5e.hand_move_init(1);
//    ros::Duration(1).sleep();
//    UR5e.hand_move_init(2);



//    ROS_INFO("Next move");
//    UR5e.lei_hand_move(3200,0);
//    ros::spin();
    //        运动到初始位置

//        ROS_INFO_STREAM("move to start position");
////    double joint_start_arr[] = { -28.91,-93.57,-106.94,-71.57,90.06,-113.22};
//        double joint_start_arr[] = {-29.72, -94.55, -105.86, -69.59, 90.00, -119.72};   //y axis rotate PI  抓取位置
//        KDL::JntArray start_joint(6);
//        for (int i = 0; i < 6; i++) {
//            start_joint(i) = joint_start_arr[i] * M_PI / 180;
//        }
//
//
//        KDL::Vector target_point_1(-0.4, -0.4, 0.5);
//        KDL::Vector target_point_2(0.5,-0.3,0.34);
//        UR5e.servoj_moveto(start_joint, 6, true);
//    for(int j = 1 ;j < 51 ;j++) {
////        int num = j;
//        UR5e.move_line_fix_end_(target_point_1,j,0.4);
//        ros::Duration(1).sleep();
//
//        UR5e.move_line_fix_end_(target_point_2,j,0.4);
//        ros::Duration(1).sleep();
//    }


//    //test force interaction
////    UR5e.force_interaction();
////    ros::Duration(10000).sleep();
//
//
////    动捕抓水果
//    KDL::Vector fruit(0.21,-0.46,0.3),hand_marker(-0.21,0.37,0.37);
//    while(! UR5e.choose_fruit(fruit))
//    {
//        ROS_INFO_STREAM("choose fruit ... ");
//        ros::Duration(0.5).sleep();
//    }
//
//
//    ros::Duration(0.2).sleep();
////    fruit[2] += 0.146 + 0.077 + 0.1;
//    UR5e.move_line(fruit,1);
//    UR5e.move_to_grasp_new_function(fruit);
//
////    UR5e.test_move_line(fruit);
//
////    UR5e.test_track_marker();
////    KDL::Vector end_point(-0.37,0.36,0.35);
//
//
////    ROS_INFO_STREAM("move to first point ");
////    ROS_INFO_STREAM("move to test move line ");
////    KDL::Vector end_point(-0.405451,-0.378451,0.487925);
////    UR5e.move_line(end_point,1);
////    ros::Duration(2).sleep();
//
////    UR5e.test_move_line(end_point);
////    ros::Duration(2).sleep();
////    KDL::Vector end_point2(0.21,-0.46,0);
////    UR5e.test_move_line(end_point2);
//
////    目前有问题 不能和ros_controol
////    ROS_INFO_STREAM("test URScript ");
////    ROS_INFO_STREAM("test free drive mode ");
////    UR5e.test_free_drive();
//
//
//
////    UR5e.robotiq_hand_move(10,5,100);
//
//
////    UR5e.servoj_sin_test();
////    UR5e.robotiq_hand_move(62,5,100);
////    UR5e.catch_bottle();
////    UR5e.robotiq_hand_move(80,5,50);
////    UR5e.move_to_xyz(0.02,0.02,0.02,2, true);
////    UR5e.robotiq_hand_move(26,5,50);
////    ros::waitForShutdown();



    ros::shutdown();
    return  1;


}

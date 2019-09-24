//
// Created by yang on 3/31/19.
//
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/CollisionObject.h>
#include "match_demo.h"
#include <moveit_visual_tools/moveit_visual_tools.h>




int main(int argc,char **argv)
{
    ros::init(argc,argv,"mymoveit_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    bool pose_orien = false;
    double hand_bias_z = 0.13, softhand = 0.125;
    std::vector<double> bottle_x_y_z;

    double hand_dis = 85.0;
    std::vector<double> rgb_bias_x_y;
    nh.param("pose_orien", pose_orien, false);
    nh.param("hand_dis", hand_dis, 56.0);
    nh.param("rgb_bias_x_y", rgb_bias_x_y, {0.0, 0.0, 0.0});

    UR_gripper ur5e_gripper(nh,"manipulator");
    ur5e_gripper.robotiq_hand_move(85,5,50);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("ur_world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();


    KDL::Frame base2ur_world;
    base2ur_world.p = KDL::Vector(-0.01 ,0.377 ,0.808);
    base2ur_world.M = KDL::Rotation::RPY(0,0,1.570796327);

    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> joint_name;
    joint_name = move_group.getJointNames();
    move_group.allowReplanning(true);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    ROS_INFO_STREAM("planning frame: "<< ur5e_gripper.move_group->getPlanningFrame(); );


    double joint_satrt_arr[] = { 1.15, -1.82, -1.81, -1.09, 1.56, -0.91 };
    std::vector<double> joint_start(joint_satrt_arr, joint_satrt_arr + 6);
    move_group.setJointValueTarget(joint_start);

    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    bool success3 = (move_group.plan(start_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group.execute(start_plan);
//    move_group.move();


    double rgb_box_position[3];
    std::vector<double> rgb_box_vec;


    if(pose_orien)
    {
            do{
                ros::spinOnce();
            }while(! ur5e_gripper.get_box_frame);

    }
    else{
        //    彩色物体位置
        do{
            ur5e_gripper.get_camera_rgb = false;
            do{
                ros::spinOnce();
            }while(!ur5e_gripper.get_camera_rgb);
            ros::Duration(1).sleep();
            rgb_box_vec = ur5e_gripper.box_rgb_vec;
            ROS_INFO_STREAM("point size  "<< ur5e_gripper.box_rgb_vec.size()/3);
            rgb_box_position[0] = ur5e_gripper.camera_box_position_rgb[0];
            rgb_box_position[1] = ur5e_gripper.camera_box_position_rgb[1];
            rgb_box_position[2] = ur5e_gripper.camera_box_position_rgb[2];
        }while(!(ur5e_gripper.box_rgb_vec.size()/3) );
    }

    double point_up_collectBox[3] = { 0.44, 0.03, 1.1 };

    ros::Duration(1).sleep();
    KDL::Frame grasp_frame = ur5e_gripper.grasp_frame_bef;
    std::vector<KDL::Frame> grasp_frame_ber_vec = ur5e_gripper.grasp_frame_bef_vec;
    ROS_INFO_STREAM("bottle number "<<grasp_frame_ber_vec.size());

    double bottle_center[] = { grasp_frame.p.x(), grasp_frame.p.y(), grasp_frame.p.z()};

    double bottle_size[] = { 0.116 , 0.033};   //易拉罐高 半径


    //    发布物体
    moveit_msgs::CollisionObject collision_object;
//    ur5e_gripper.add_box(collision_object,bottle_size,bottle_center,grasp_frame_urWorld, "box1","ur_world");

    ros::Duration(2).sleep();

    if(pose_orien){
        for(unsigned int i=0;i<grasp_frame_ber_vec.size();i++)
        {
            KDL::Frame grasp_frame_urWorld = grasp_frame_ber_vec[i];
            //    根据瓶子的方向旋转一定角度 初始位置已经在joint[5] 中心附近

            std::vector<double> joint_value = move_group.getCurrentJointValues();
            geometry_msgs::Pose pose_now = move_group.getCurrentPose().pose;
            KDL::Rotation boxRot_bias_y;
            boxRot_bias_y = KDL::Rotation::Quaternion(pose_now.orientation.x, pose_now.orientation.y,
                                                      pose_now.orientation.z, pose_now.orientation.w);
            KDL::Vector ur_tool_y = boxRot_bias_y.UnitY();
            Eigen::Vector3d ur_tool_calc(ur_tool_y.x(), ur_tool_y.y(), 0);

            KDL::Vector box_y = grasp_frame_urWorld.M.UnitY();
            Eigen::Vector3d box_y_calc(box_y.x(), box_y.y(), 0);
            double rad_grasp_goal = acos( ur_tool_calc.dot(box_y_calc)/
                                                  ( sqrt(ur_tool_calc.dot(ur_tool_calc)) * sqrt(box_y_calc.dot(box_y_calc)) ) );
            if(rad_grasp_goal > M_PI_2) {
                rad_grasp_goal = M_PI - rad_grasp_goal;
                if( (ur_tool_calc.cross(box_y_calc) )[2] < 0) {
                    joint_value[5] -= rad_grasp_goal;
                } else joint_value[5] += rad_grasp_goal;
            } else {
                if( (ur_tool_calc.cross(box_y_calc) )[2] < 0) { joint_value[5] += rad_grasp_goal; }
                else joint_value[5] -= rad_grasp_goal;

            }
            ROS_INFO_STREAM("rad_grasp_goal "<<rad_grasp_goal);

            move_group.setJointValueTarget(joint_value);
            move_group.move();

            ros::Duration(2).sleep();

//        位姿抓取
            double point_up_bottle_plan[] ={
                    0,0,0.15,0,
                    grasp_frame_urWorld.p.x(), grasp_frame_urWorld.p.y(), grasp_frame_urWorld.p.z() + 0.15 + softhand,  1,
                    0,-0.02,-0.15 + softhand - 0.040,0 };

            ur5e_gripper.com_plan_CartsianPath(point_up_bottle_plan,3);

            ur5e_gripper.robotiq_hand_move(32,5,50);
            ros::Duration(4).sleep();

            double back_point_plan[] = {0,0,0.15,0, point_up_collectBox[0],point_up_collectBox[1],point_up_collectBox[2],1};
            ur5e_gripper.com_plan_CartsianPath(back_point_plan,2);

            ur5e_gripper.robotiq_hand_move(85,5,50);

            ros::Duration(3).sleep();
        }
    }


//    直线抓取路径
    else
    {
        for(unsigned int i=0;i < rgb_box_vec.size();i += 3)
        {
            //    彩色

            double point_up_bottle_plan_rgb[] = {
                    0,0,0.15,0,
                    rgb_box_vec[i], rgb_box_vec[i+1], rgb_box_vec[i+2] + 0.15 + softhand,  1,
                    0.0, 0.02, -0.15 + softhand - 0.035 + rgb_bias_x_y[2],0 };

            ur5e_gripper.com_plan_CartsianPath(point_up_bottle_plan_rgb,3);

//            while(!ur5e_gripper.myo_grasp) { }
            ur5e_gripper.robotiq_hand_move(hand_dis,5,50);
            ros::Duration(4).sleep();
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pick the fruit ");

//            do{
//                ros::spinOnce();
//            }while(!ur5e_gripper.capture_point_ready);

//            std::vector<double> point_vec;
//            point_vec = ur5e_gripper.capture_point_vec;
//            point_up_collectBox[0] = point_vec[0];
//            point_up_collectBox[1] = point_vec[1];
//            point_up_collectBox[2] = point_vec[2] + 0.05;


            double back_point_plan[] = {0,0,0.15,0, point_up_collectBox[0],point_up_collectBox[1],point_up_collectBox[2],1};
            ur5e_gripper.com_plan_CartsianPath(back_point_plan,2);

//            while(!ur5e_gripper.myo_grasp) { }
            ur5e_gripper.robotiq_hand_move(85,5,50);

            ros::Duration(3).sleep();
        }
    }

    ros::waitForShutdown();
    return 1;
}
//
// Created by yang on 4/15/19.
//

#ifndef CATCH_BOTTLE_MYMOVEIT_H
#define CATCH_BOTTLE_MYMOVEIT_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "robotiq_85_msgs/GripperCmd.h"
#include "robotiq_85_msgs/GripperStat.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

class UR_gripper{

public:
    UR_gripper(ros::NodeHandle _nh,std::string PLANNING_GROUP){
        move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        sub_hand_state = _nh.subscribe("/gripper/stat",1 , &UR_gripper::subHandStatesCB,this);
        pub_hand_cmd_ = _nh.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd",1);
    }


    void subHandStatesCB(robotiq_85_msgs::GripperStat state){
        hand_state_ = state;
        finish_hand_sub_ = true;
    }

    void robotiq_hand_move(float position,float vel, float force){
        ROS_INFO_STREAM("hand move, position="<<position<<"  vel="<<vel<<"  force="<<force);
        robotiq_85_msgs::GripperCmd hand_cmd;
        position = position/1000;
        vel = vel/1000;
        force = force/1000;
        if (position < 0 || position>0.086 || vel>0.02 ||vel<-0.02 || force>255 ||force<0){
            ROS_ERROR("The command of the Robotiq is out of range");
            return;
        }
//        force_update_all_state();
        if(true ){
            hand_cmd.position = position;   // scale 0-0.085mm
            hand_cmd.speed = vel;
            hand_cmd.force = force;
            pub_hand_cmd_.publish(hand_cmd);
        }
    }

    void force_update_all_state(){
        finish_hand_sub_ = false;
        do{
            ros::spinOnce();
        }while(!finish_hand_sub_);
    }

//    轨迹重定义
    void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale)
    {
        int n_joints = plan.trajectory_.joint_trajectory.joint_names.size();

        for(unsigned int i=0; i<plan.trajectory_.joint_trajectory.points.size(); i++)
        {
            plan.trajectory_.joint_trajectory.points[i].time_from_start *= 1/scale;

            for(int j=0; j<n_joints; j++)
            {
                plan.trajectory_.joint_trajectory.points[i].velocities[j] *= scale;
                plan.trajectory_.joint_trajectory.points[i].accelerations[j] *= scale*scale;
            }
        }
    }

//    添加障碍物




    void add_box(moveit_msgs::CollisionObject& collision_object,double bottle_height,std::vector<double> bottle_x_y_z,
            std::string object_name ,std::string reference_frame)
    {
//        moveit_msgs::CollisionObject collision_object;move_group->getPlanningFrame();
        collision_object.header.frame_id = reference_frame;
        collision_object.id = object_name;


        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = bottle_height;
        primitive.dimensions[1] = bottle_height;
        primitive.dimensions[2] = bottle_height;

        geometry_msgs::Pose bottle_pose;
        bottle_pose.orientation.w = 1.0;
        bottle_pose.position.x = bottle_x_y_z[0];
        bottle_pose.position.y = bottle_x_y_z[1];
        bottle_pose.position.z = bottle_x_y_z[2] + bottle_height/2;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(bottle_pose);
        collision_object.operation = collision_object.ADD;

        ROS_INFO_NAMED("tutorial", "Add an object into the world");
        planning_scene_interface.applyCollisionObject(collision_object);
        //    planning_scene_interface.addCollisionObjects(collision_objects);   //需要睡眠一下 否则rviz可能没有物体

//        return collision_object;
    }

    void plan_CartesianPath()
    {
        geometry_msgs::Pose target_pose3 = move_group->getCurrentPose().pose;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose3);

        target_pose3.position.z += 0.05;
        waypoints.push_back(target_pose3);  // down

        target_pose3.position.x += 0.1;
        target_pose3.position.y += 0.15;
        waypoints.push_back(target_pose3);

        target_pose3.position.z -= 0.05;
        waypoints.push_back(target_pose3);

        move_group->setMaxVelocityScalingFactor(0.1);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;  //跳跃阀值 有冗余自由度的话可能会有问题
        const double eef_step = 0.01;   //步长1mm
        int maxtries = 100;   //最大尝试次数
        int attempt = 0;
        double fraction = 0.0;
        while(fraction < 1.0 && attempt < maxtries)
        {
            fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempt++;
            if(attempt %10 == 0)
                ROS_INFO("still trying after %d attempt...",attempt);
        }

        if(fraction ==1)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group->execute(plan);
            ros::Duration(3).sleep();
        }

        ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    }
private:


    moveit::planning_interface::MoveGroupInterface* move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Subscriber sub_hand_state;
    ros::Publisher pub_hand_cmd_;

    robotiq_85_msgs::GripperStat hand_state_;
    bool finish_hand_sub_;
};



#endif //CATCH_BOTTLE_MYMOVEIT_H

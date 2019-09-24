//
// Created by zh on 18-11-3.
//

#ifndef PROJECT_CATCH_BOTTLE_H
#define PROJECT_CATCH_BOTTLE_H


#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include "robotiq_85_msgs/GripperCmd.h"
#include "robotiq_85_msgs/GripperStat.h"
#include <vector>

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>

#include "ros_myo/EmgArray.h"

using namespace Eigen;
//simulink2C
#include "../simulink2C/end_point_filter/end_point_filter.h"
#include "../simulink2C/force_interaction/force_interaction.h"

class UR{

    typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;

public:
    UR(ros::NodeHandle _nh, const std::string & _urdf_param, const std::string & _chain_start, const std::string & _chain_end, double _timeout,std::string UR_name_prefix, std::string hand_name_prefix){
        double eps = 1e-5;
        nh_ = _nh;


        p_tracik_solver_ = new TRAC_IK::TRAC_IK(_chain_start, _chain_end, _urdf_param, _timeout, eps); //反解
        KDL::Chain chain;
        bool valid = p_tracik_solver_->getKDLChain(chain);
        p_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain);  //正解
        //init
        joint_names_.push_back("shoulder_pan_joint");
        joint_names_.push_back("shoulder_lift_joint");
        joint_names_.push_back("elbow_joint");
        joint_names_.push_back("wrist_1_joint");
        joint_names_.push_back("wrist_2_joint");
        joint_names_.push_back("wrist_3_joint");
        current_JntArr_.resize(6);
        for(int i=0;i<6;++i)
        {
            if (i==1||i==3)
                current_JntArr_(i) = -M_PI/2;
            else if(i==0)
                current_JntArr_(i) = M_PI;
            else
                current_JntArr_(i) = 0;
        }
        bsub_ = false;
        bstart_cal_ = false;

        sub_=nh_.subscribe(UR_name_prefix+"/joint_states", 1, &UR::subJointStatesCB, this);
        sub_hand_ = nh_.subscribe("gripper/stat",1 , &UR::subHandStatesCB,this);
        pub_hand_cmd_ = nh_.advertise<robotiq_85_msgs::GripperCmd>("gripper/cmd",1);

        hand_cmd = nh_.advertise<std_msgs::String>("hand_cmd",1);

        pub_free_drive_ = nh_.advertise<std_msgs::String>("UR5e/ur_driver/URScript",1);

        pub_end_point_ = nh_.advertise<geometry_msgs::Point>("end_point",1);

        sub_markers_ = nh_.subscribe("vis_markers", 1, &UR::subMotionCaptureCB, this);
        sub_myo_hand_ = nh_.subscribe("/myo_raw/myo_emg",1,&UR::sub_myo_handCall,this);
        ur_wrench_ = nh_.subscribe(UR_name_prefix+"/wrench", 1,&UR::sub_wrench,this);
        ROS_INFO("sub done");

        //action client
//        client_ = new Client(UR_name_prefix+"/follow_joint_trajectory", true);
//        client_->waitForServer(ros::Duration());
        // servoj client

        client_servoj_ = new Client(UR_name_prefix+"/pos_based_pos_traj_controller/follow_joint_trajectory",true);
        ROS_INFO("wait for servoj client");
        client_servoj_->waitForServer(ros::Duration());
        ROS_INFO("servoj clent connect!");

//        ROS_INFO_STREAM("begin to innitialize wrench " );
//        bsub_wrench_ = false;
//        do{
//            ros::spinOnce();
//        }while(!bsub_wrench_);
//        start_wrench = wrench_base_;

        wrench_repair_.force = KDL::Vector(0,0,0);
        wrench_repair_.torque = KDL::Vector(0,0,0);

    }

    ~UR(){
        delete p_tracik_solver_;
        delete client_;
        delete client_servoj_;
    }


    void draw_sin(bool bLeftArm)
    {
        int dir = bLeftArm?-1:1;
        ROS_INFO("draw sin left");
        KDL::Chain chain;
        bool valid = p_tracik_solver_->getKDLChain(chain);

        //start jntArr
        uint nbr = chain.getNrOfJoints();
        KDL::JntArray p(nbr);
        p = current_JntArr_;
//    for(int i=0;i<20;++i)
//    {
//      if(bsub_){
//        p = current_JntArr_;
//        break;
//       }
//      else
//      {
//         ros::Duration(0.5).sleep();
//         ROS_INFO_STREAM("loop"<<i);
//      }
//   }
        for(unsigned int i=0;i<nbr;++i)
        {
            ROS_INFO_STREAM("name: "<<chain.getSegment(i).getJoint().getName()<<"   pos: "<<p(i)<<"\n");
        }

        //ee pose
        KDL::Frame end_effector_pose;
        end_effector_pose.M.data[0] = 1;
        end_effector_pose.M.data[1] = 0;
        end_effector_pose.M.data[2] = 0;
        end_effector_pose.M.data[3] = 0;
        end_effector_pose.M.data[4] = 1;
        end_effector_pose.M.data[5] = 0;
        end_effector_pose.M.data[6] = 0;
        end_effector_pose.M.data[7] = 0;
        end_effector_pose.M.data[8] = 1;

        //cal
        std::vector<KDL::JntArray>  vResult;
        KDL::JntArray result;
        int i=0, range=50;
        for(i=-range;i<range;++i)
        {
            //     ROS_INFO("Pt %d ", i);
            end_effector_pose.p.data[0] = dir*(i*M_PI/(30*range)+0.1);
            end_effector_pose.p.data[1] = 0.325;
            end_effector_pose.p.data[2] = 0.05*sin(i*M_PI/range)+0.435;
            p_tracik_solver_->CartToJnt(p, end_effector_pose, result);
            p = result;
            vResult.push_back(result);
        }
        //reverse
        for(i=range;i>-range;--i)
        {
            //     ROS_INFO("Pt %d ", i);
            end_effector_pose.p.data[0] = dir*(i*M_PI/(30*range)+0.1);
            end_effector_pose.p.data[1] = 0.325;
            end_effector_pose.p.data[2] = -0.05*sin(i*M_PI/range)+0.435;
            p_tracik_solver_->CartToJnt(p, end_effector_pose, result);
            p = result;
            vResult.push_back(result);

        }

//   // pub test
//    ros::Publisher pub=nh_.advertise<sensor_msgs::JointState>("joint_states",1);
//    ros::Rate rate(15);

//    while(ros::ok()){
//    KDL::JntArray jntArr;
//    for( i=0;i<vResult.size();++i)
//    {
//      sensor_msgs::JointState jointMsg;
//      jointMsg.header.stamp = ros::Time::now();

//      jntArr = vResult[i];
//      int n = jntArr.rows()*jntArr.columns();
//      for(int x=0;x<n;++x)
//      {
//        jointMsg.name.push_back(chain.getSegment(x).getJoint().getName());
//        jointMsg.position.push_back(jntArr(x));
//      }

//      pub.publish(jointMsg);
//      ros::spinOnce();
//      rate.sleep();
//    }
//    }
//    return;//test

        //trajectory goal
        control_msgs::FollowJointTrajectoryGoal g;
        //head
//    g.trajectory.header.frame_id = "base_link";
        g.trajectory.header.stamp = ros::Time::now();
        //names
        for(unsigned i=0;i<nbr;++i)
        {
            std::string name = chain.getSegment(i).getJoint().getName();
            g.trajectory.joint_names.push_back(name);
            ROS_INFO_STREAM("joint name : "<<name);
        }

        //loop result
        KDL::JntArray jntArr;
        double dur = 5.0;
        for(unsigned i=0;i<vResult.size();++i)
        {
            jntArr = vResult[i];
            //pt
            trajectory_msgs::JointTrajectoryPoint pt;
            int n=jntArr.rows()*jntArr.columns();
            for(int x=0;x<n;++x)
            {
                pt.positions.push_back(jntArr(x));
                pt.velocities.push_back(0);
            }

            pt.time_from_start = ros::Duration(dur);
            dur += 0.05;
            g.trajectory.points.push_back(pt);
            ROS_INFO_STREAM("  joint pos 1: "<<pt.positions[0]<<" joint pos 2: "<<pt.positions[1]<<" joint pos 3: "<<pt.positions[2]<<"  joint pos 4: "<<pt.positions[3]<<"  joint pos 5: "<<pt.positions[4]<<"  joint pos 6: "<<pt.positions[5]);
//      break;//test
        }

        client_->sendGoal(g);
        client_->waitForResult(ros::Duration());

//    if(client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
//      client_->cancelGoal();
//    }

    }



    void moveto_joints(KDL::JntArray target_jnt, double time){
//        ROS_INFO("move to joints");
        //运行前需要ros::spinOnce 来更新当前关节角
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(bur_sub_==false);

        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");

        for(int x=0;x<6;++x)
        {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);
        g.trajectory.points.push_back(p0);

        for(int x=0;x<6;++x)
        {
            p1.positions.push_back(target_jnt(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_->sendGoal(g);
        client_->waitForResult(ros::Duration());

    }

    void servoj_sin_test(){
        std::vector<double> gravity_down_joints;
        nh_.getParam("gravity_down_joints", gravity_down_joints);
        KDL::JntArray gravity_down_jnt(6);
        for (int i = 0; i < 6; ++i) {
            gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
        }
        ROS_INFO_STREAM("t0 ");
        servoj_moveto(gravity_down_jnt, 5, true);

        ROS_INFO_STREAM("t1 ");
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(!bur_sub_);
        KDL::Frame original_P = frame_wrist3_base_;

        ros::Rate loop_rate(500);
        uint64_t nsec0 = ros::Time::now().toNSec();
        while(ros::ok()){
            ros::spinOnce();
            ROS_INFO_STREAM("z " << frame_wrist3_base_.p.z() );
            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
            double time_now_d = time_now / 1e9;
            KDL::Frame p1 = original_P;
            KDL::JntArray q1;
            q1.resize(6);
            p1.p.data[2] = original_P.p.data[2]+ 0.05*sin(2*M_PI/5*time_now_d);
            p_tracik_solver_->CartToJnt(current_JntArr_,p1,q1);
            servoj_moveto(q1,0.002,false);

            //record
            std::ofstream file3;
            std::ostringstream oss;
            int name_suffix = 1;
            oss << name_suffix;
            std::string filename = oss.str() + "_servoj_tracking_real_robot";
            std::string file = "/home/zd/project/catch_bottle/catch_control/record_servoj_sim/" + filename + ".txt";
            file3.open(file.c_str(), std::ios::app);
            file3 << time_now_d << " ";
            file3 << p1.p.data[0]<< " " <<p1.p.data[1] << " " << p1.p.data[2]<< " "
                  << end_point_.data[0]<< " " << end_point_.data[1] << " " <<end_point_.data[2];
            file3 << std::endl;
            file3.close();
            loop_rate.sleep();
            if (time_now_d>20)
                break;
        }



    }

    void servoj_moveto(KDL::JntArray target, double time,bool wait_for_D) {
        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");


        for(int x=0;x<6;++x)
        {
            p1.positions.push_back(target(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_servoj_->sendGoal(g);
        if(wait_for_D)
            ros::Duration(time).sleep();

    }

    void force_interaction(){
        // repair wrench
        ROS_INFO_STREAM("begin to initialize wrench " );
        bsub_wrench_ = false;
        do{
            ros::spinOnce();
        }while(!bsub_wrench_);
        wrench_repair_ = wrench_base_;
//     ros::spin();
        // test force interaction controller
//        x_kd = 0.1;
//        x_ki = 0;
//        x_kd = 0;   // xyz PID parameters
//        r_kp = 0;
//        r_ki = 0;
//        r_kd = 0;   // rpy PID parameters
        ForceInteraction controller1;
        controller1.initialize();
        double force_base[6]={0,0,0,0,0,0};
        double delta_x[3] = {0,0,0};
        double delta_rpy[3] = {0,0,0};

        sub_ur_update();
        KDL::Frame target_frame = frame_wrist3_base_;
        KDL::Frame start_frame = frame_wrist3_base_;
        KDL::JntArray target_jnt(6);
        uint64_t sec0 = ros::Time::now().toSec();
        ROS_INFO_STREAM("start force  interaction " );
        ros::Rate loop_rate(500);

        while(ros::ok()){
            ros::spinOnce();
            // contact force protection
            double sum_force = wrench_base_.force.Norm();
            if (sum_force>50){
                ROS_ERROR("contact force is too big. stop moving");
                break;
            }
            for (int i = 0; i <3 ; ++i) {
                force_base[i]   = wrench_base_.force.data[i];   //六维力力矩传感器 force（前三个）
                force_base[i+3] = wrench_base_.torque.data[i];  //torque
            }

            target_frame.p = frame_wrist3_base_ .p;    // consider xyz without rpy rotation.
            double time_now_d = ros::Time::now().toSec() - sec0;
            controller1.step(force_base,delta_x,delta_rpy);





            for (int j = 0; j < 3; ++j) {
                if (abs(wrench_base_.force.data[j])<5) {
                    delta_x[j] = 0;
//                    controller1.initialize();
                }
                target_frame.p.data[j] +=delta_x[j]/1000.0;    // mm to m
            }
            ROS_INFO_STREAM("delta_xyz: "<< delta_x[0]<<" "<< delta_x[1]<<" "<< delta_x[2]<<" "
                                         << wrench_base_.force.data[0]<<" "<< wrench_base_.force.data[1]
                                         <<" "<<wrench_base_.force.data[2]<<" ");
            move_line_dynamic(target_frame.p,0.1);
//            p_tracik_solver_->CartToJnt(current_JntArr_,target_frame,target_jnt);
//            servoj_moveto(target_jnt,0.002,false);
            loop_rate.sleep();
        }
    }

// control Robotiq in [mm]
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
        force_update_all_state();
        if(true ){
            hand_cmd.position = position;   // scale 0-0.085mm
            hand_cmd.speed = vel;
            hand_cmd.force = force;
            pub_hand_cmd_.publish(hand_cmd);

        }

    }

    void HandMoveToFixedPosition(double Ang)
    {
        int dir;
        double displace , step;
        double time;
        all_angle.push_back(Ang);
        if(all_angle.size() == 1)
        {
            //第一个值,对应张开
            dir = 1;
            displace = TransAngleToDisplace(all_angle[0]) - TransAngleToDisplace(0);
            step = ceil(TransDisplaceToStep(displace));//转换成整数arduino中写的程序需要
            std::cout << "displace: " << displace << std::endl << "step: " << step << std::endl << "dir: " << dir <<  std::endl;
            lei_hand_move(step,dir);
            time = step / 3200 + 5;
            std::cout << "time: " << time << std::endl;
            ros::Duration(time).sleep();
        }
        else
        {
            //最后一个数同前一个数做比较 大于对应张开
            if(all_angle[all_angle.size() - 1] > all_angle[all_angle.size() - 2])
            {
                dir = 1;
                displace = TransAngleToDisplace(all_angle[all_angle.size() -1 ]) - TransAngleToDisplace(all_angle[all_angle.size() -2 ]);
                step = ceil(TransDisplaceToStep(displace));
                std::cout << "displace: " << displace << std::endl << "step: " << step << std::endl << "dir: " << dir << std::endl;
                lei_hand_move(step,dir);
                time = step / 3200 + 5;
                std::cout << "time: " << time << std::endl;
                ros::Duration(time).sleep();
            }
            else
            {
                dir = 0;
                displace = TransAngleToDisplace(all_angle[all_angle.size() -2 ]) - TransAngleToDisplace(all_angle[all_angle.size() -1 ]);
                step = ceil(TransDisplaceToStep(displace));
                std::cout << "displace: " << displace << std::endl << "step: " << step << std::endl << "dir: " << dir << std::endl;
                lei_hand_move(step,dir);
                time = step / 3200 + 4;
                std::cout << "time: " << time << std::endl;
                ros::Duration(time).sleep();
            }
        }
        ROS_INFO("Complete a moving");
    }
    double TransDisplaceToStep(double Displacement)
    {

        double Lead = 2;double StepAngle = 1.8;double Subdivision = 16;
        double Step;
        Step = (360 * Displacement) / (Lead * (StepAngle / Subdivision));
//        Displacement = ((StepAngle / Subdivision) * Step * Lead) / 360;
        return Step;
    }
    double TransAngleToDisplace (double angle)

    {
        double Ax = 24;
        double Ay = 20;
        double a = 40;
        double B[2] = {53.5 ,44};
        double C[2] = {28.93,61.2};
        double angle_ = ((angle / 180) * M_PI) / 2;
        MatrixXd rot(2,2);
        rot(0,0) = cos(angle_);rot(0,1) = sin(angle_);
        rot(1,0) = -sin(angle_);rot(1,1) = cos(angle_);
        VectorXd v(2);
//        std::cout << "rot = " << std::endl << rot << std::endl;
        v(0) = C[0] - B[0];v(1) = C[1] - B[1];
//        std::cout << "v = " << std::endl << v << std::endl;
        VectorXd BC_ = rot * v;
//        std::cout << "BC_ = " << std::endl << BC_ << std::endl;
        VectorXd C_(2);
        C_(0) = B[0] + BC_(0); C_(1) = B[1] + BC_(1);
//        std::cout << "C_ = " << std::endl << C_ << std::endl;
        double displacement = (C_[1] - sqrt(pow(a,2) -pow((C_(0) - Ax),2))) - Ay;
//        ROS_INFO("displacement:");
//        ROS_INFO_STREAM(displacement);
        return displacement;
//        double rot[2][2] ={cos(angle_) , sin(angle_) ; -sin(angle_) , };
    }

    void hand_ExtStop()
    {
        ROS_INFO("ExtStop");
        hand_move_init(2);
        ros::Duration(3).sleep();
        all_angle.clear();
    }
    void hand_init()
    {
        //注意Time
        //Step 3200 = Displacement 2mm = time 1 sec
        ROS_INFO("Waiting for intialization");
        hand_move_init(0);
        ros::Duration(3).sleep();
        hand_move_init(1);
        ros::Duration(14).sleep();
        hand_move_init(2);
        ros::Duration(3).sleep();
        hand_move_init(3);
        ros::Duration(14).sleep();
        ROS_INFO("Complete initialization");
        all_angle.clear();
    }

    void hand_move_init(int num){
        std_msgs::String  l_hand_cmd;
        std::ostringstream oss;
        oss << num;
        l_hand_cmd.data = oss.str();
        hand_cmd.publish(l_hand_cmd);
    }
    void lei_hand_move(double step, int dir){
        std_msgs::String l_hand_cmd;
        std::ostringstream oss,ss;
        oss << step;
        ss << dir;
        l_hand_cmd.data = oss.str() + " " + ss.str();
        std::cout << l_hand_cmd.data << std::endl;
        hand_cmd.publish(l_hand_cmd);
    }

    void catch_bottle(){
        ROS_INFO_STREAM("begin to catch the bottle.");
        robotiq_hand_move(85,10,50);   // open hand
        move_to_xyz(-0.059, 0.341, 0.433, 3, false);//home点
        //竖直的瓶子
        do{
        force_update_all_state();
        }while(check_markers_fine(first_body_markers_[0]));

        KDL::Vector A0 = first_body_markers_[0];

        KDL::Frame bottle_target;
        KDL::JntArray bottle_target_jnt(6);
        bottle_target.M = KDL::Rotation::RPY(-M_PI/2,-M_PI/2,0);  //base_link的z轴旋转90度
        bottle_target.p = A0 + KDL::Vector(0,-0.15-0.05,-0.13);


        p_tracik_solver_->CartToJnt(current_JntArr_,bottle_target,bottle_target_jnt);
        moveto_joints(bottle_target_jnt,3);
        move_to_xyz(0, 0.05, 0, 2, true);
        ros::Duration(3).sleep();
        do{
            force_update_all_state();
        }while(check_markers_fine(first_body_markers_[0]));
        double bottle_move = 0.03;


        while (bottle_move>0.02){
            robotiq_hand_move(85,10,50);   // open hand
            do{
                force_update_all_state();
            }while(check_markers_fine(first_body_markers_[0]));
            KDL::Vector A1 = first_body_markers_[0];
            A1 = first_body_markers_[0];
            bottle_move = sqrt(pow(A1.data[0]-A0.data[0],2)+  pow(A1.data[1]-A0.data[1],2)+pow(A1.data[2]-A0.data[2],2) );
            A0 = A1;
            move_to_xyz(0, -0.1-0.05, 0.2, 2, true);
            bottle_target.p = A1 + KDL::Vector(0,-0.15-0.05,-0.13);
            p_tracik_solver_->CartToJnt(current_JntArr_,bottle_target,bottle_target_jnt);
            moveto_joints(bottle_target_jnt,3);        robotiq_hand_move(85,10,50);   // open hand

            move_to_xyz(0, 0.05, 0, 2, true);
            robotiq_hand_move(56,5,50);   //catch  close hand 56mm
            ros::Duration(2).sleep();
            do{
                force_update_all_state();
            }while(check_markers_fine(first_body_markers_[0]));
            A1 = first_body_markers_[0];
            bottle_move = sqrt(pow(A1.data[0]-A0.data[0],2)+  pow(A1.data[1]-A0.data[1],2)+pow(A1.data[2]-A0.data[2],2) );
        }


        //向上并运送到指定位置
        force_update_all_state();
        do{
            ros::spinOnce();
        }while(!b_hand_sub_);
        if (hand_state_.is_moving){
            //向上 200mm
            ros::Duration(1).sleep();
        }
        ros::Duration(2).sleep();
        move_to_xyz(0, 0, 0.2, 3, true);
        // 放到固定位置 -0.2064 0.402 bottle_target.data[2];
        move_to_xyz(-0.2064,0.402,bottle_target.p.data[2]+0.2, 3, false);
        //向下0.2
        move_to_xyz(0, 0, -0.2, 3, true);
        robotiq_hand_move(85,5,50);   // put
        ros::Duration(1).sleep();
        move_to_xyz(0, -0.1, 0, 2, true);
        move_to_xyz(0, 0, 0.2, 3, true);
        move_to_xyz(-0.059, 0.341, 0.433, 2, false);//home点









    }

    void calibrate_motioncapture_to_UR(std::string path){

        ROS_INFO_STREAM("cal begin");
        bstart_cal_ = true;
        ros::spin();


    }

    void force_update_all_state(){
//        b_hand_sub_ = false;
        bur_sub_ = false;
        bmotion_capture_sub_ = false;
        b_hand_sub_ = false;
        do{
            ros::spinOnce();
        }while(!bur_sub_ );
    }
    bool check_markers_fine(KDL::Vector point){
        if (fabs(point.data[0])<0.4&& point.data[0]>0.46&&point.data[1]< 0.85 &&point.data[1]>0.1 &&point.data[2]< 0.7)
            return true;
        else
            return false;

    }

    void move_to_xyz(double x,double y, double z,double time,bool b_relative){
        force_update_all_state();
        KDL::Frame next_frame = frame_wrist3_base_;
        KDL::JntArray next_jnts(6);
        if(b_relative){
            ROS_INFO_STREAM("move to position relative to current point   x="<<x<<"   y="<<y<<"  z"<<z);
            next_frame.p.data[0] += x;
            next_frame.p.data[1] += y;
            next_frame.p.data[2] += z;

        } else{
            ROS_INFO_STREAM("move to fixed position     x="<<x<<"   y="<<y<<"  z"<<z);
            next_frame.p.data[0] = x;
            next_frame.p.data[1] = y;
            next_frame.p.data[2] = z;
        }
        p_tracik_solver_->CartToJnt(current_JntArr_,next_frame,next_jnts);
        moveto_joints(next_jnts,time);
    }

    void move_line(KDL::Vector hand_point, double line_vel, KDL::Frame start_frame, KDL::Frame &frame_compu)
    {

        KDL::Frame next_goal = frame_wrist3_base_;
        KDL::JntArray next_jnts(6);
        next_goal.M = start_frame.M;
        next_goal.p = frame_wrist3_base_.p + 0.002 * line_vel *
                (hand_point - start_frame.p)/(hand_point - start_frame.p).Norm();  //Norm
        p_tracik_solver_->CartToJnt(current_JntArr_,next_goal,next_jnts);
        frame_compu = next_goal;
        servoj_moveto(next_jnts,0.002,false);


//        ROS_INFO_STREAM("servoj "<<hand_point[0]<<" "<<hand_point[1]<<" "<<hand_point[2] );
    }

//    直线到固定终点
    void move_line(KDL::Vector end_point, double time)
    {
        sub_ur_update();   // 强制更新关节角度
        KDL::Vector start_point = frame_wrist3_base_.p;
        double dis_start_end = (end_point - start_point).Norm();
        KDL::Vector unit_vec = (end_point - start_point) / dis_start_end;
        KDL::Vector step_start_end_vec = unit_vec * dis_start_end / time * 0.002;

        KDL::Frame next_goal_frame = frame_wrist3_base_;
        KDL::JntArray next_joints(6);

        ros::Rate loop_rate(500);
        while(ros::ok() && (next_goal_frame.p - end_point).Norm() > 0.001)
        {
            ros::spinOnce();    // 不知道是否更新  需注意
            next_goal_frame.p += step_start_end_vec;
            p_tracik_solver_->CartToJnt(current_JntArr_,next_goal_frame,next_joints);
            servoj_moveto(next_joints,0.002,false);
            loop_rate.sleep();
        }

//        ROS_INFO_STREAM("servoj "<<hand_point[0]<<" "<<hand_point[1]<<" "<<hand_point[2] );
    }


    void move_line_old(KDL::Vector hand_point, double line_vel )
    {
//        一定要确保回调函数 调用
//        ros::AsyncSpinner spinner(3);
//        spinner.start();

        KDL::Frame next_goal,tool = frame_wrist3_base_;
        KDL::JntArray next_jnts(6);
        next_goal.M = tool.M;
        next_goal.p = tool.p + 0.002 * line_vel *
                               (hand_point - tool.p)/(hand_point - tool.p).Norm();
        p_tracik_solver_->CartToJnt(current_JntArr_,next_goal,next_jnts);
        servoj_moveto(next_jnts,0.002,false);

    }

//    目前只能跟踪一次
    void move_line_dynamic(KDL::Vector end_point, double vel=0.2){
        sub_ur_update();
        static KDL::Frame start_frame =  frame_wrist3_base_;
        KDL::Vector start_point = start_frame.p;

        ros::Rate loop_rate(500);

        double dis_start_end = (end_point - start_point).Norm();
        KDL::Vector unit_vec = (end_point - start_point) / dis_start_end;
        KDL::Vector step_start_end_vec = 0.002 * vel * unit_vec;

        KDL::Frame next_goal_frame = start_frame;
        KDL::JntArray next_joints(6);
        next_goal_frame.p = start_frame.p + step_start_end_vec;
        p_tracik_solver_->CartToJnt(current_JntArr_,next_goal_frame,next_joints);
        servoj_moveto(next_joints,0.002,false);
        start_frame.p = next_goal_frame.p;
        loop_rate.sleep();
    }

    void move_line_dynamic(KDL::Vector end_point, std::vector<double> &output , double vel=0.2){
        sub_ur_update();
        static KDL::Frame start_frame =  frame_wrist3_base_;
        KDL::Vector start_point = start_frame.p;

        ros::Rate loop_rate(500);

        double dis_start_end = (end_point - start_point).Norm();
        KDL::Vector unit_vec = (end_point - start_point) / dis_start_end;
        KDL::Vector step_start_end_vec = 0.002 * vel * unit_vec;

        KDL::Frame next_goal_frame = start_frame;
        KDL::JntArray next_joints(6);
        next_goal_frame.p = start_frame.p + step_start_end_vec;
        p_tracik_solver_->CartToJnt(current_JntArr_,next_goal_frame,next_joints);
        servoj_moveto(next_joints,0.004,false);
        start_frame.p = next_goal_frame.p;
        output.clear();
        for(int i = 0; i < 3; i++ )
            output.push_back(start_frame.p.data[i]);
        loop_rate.sleep();
    }



    void move_line_fix_end(KDL::Vector end_point, double vel = 0.25){
        sub_ur_update();
        std::ofstream File;
        std::ostringstream oss;
        int name_suffix = 1;
        oss << name_suffix;
        std::string filename = oss.str() + "_real_tracking";
        std::string file = "/home/dzf/match_copy-master/catch_control/record_servoj_sim/" + filename + ".txt";
        File.open(file.c_str(),std::ios::app);
        std::vector<double> output;
        while ( (frame_wrist3_base_.p - end_point).Norm() > 0.001 )
        {
            move_line_dynamic(end_point,output,vel);
            File << output[0] << " " << output[1] << " " << output[2] << " "
                 << end_point_[0]<< " " << end_point_[1] << " " << end_point_[2] << "\n";
        }
        File.close();
//        record

    }

    void move_line_fix_end_(KDL::Vector end_point,int num, double vel = 0.25){
        sub_ur_update();
        std::ofstream File;
        std::ostringstream oss;
        int name_suffix = num;
        oss << name_suffix;
        std::string filename = oss.str() + "_real_tracking";
        std::string file = "/home/dzf/match_copy-master/catch_control/record_servoj_sim/" + filename + ".txt";
        File.open(file.c_str(),std::ios::app);
        std::vector<double> output;
        while ( (frame_wrist3_base_.p - end_point).Norm() > 0.001 )
        {
            move_line_dynamic(end_point,output,vel);
            File << output[0] << " " << output[1] << " " << output[2] << " "
                 << end_point_[0]<< " " << end_point_[1] << " " << end_point_[2] << "\n";
        }
        File.close();
//        record

    }

    int move_to_grasp_new_function(KDL::Vector fruit){
//        ros::AsyncSpinner spinner(8);
//        spinner.start();

        // repair wrench
        ROS_INFO_STREAM("begin to innitialize wrench " );
        bsub_wrench_ = false;
        do{
            ros::spinOnce();
        }while(!bsub_wrench_);
        wrench_repair_ = wrench_base_;

        ForceInteraction controller1;
        controller1.initialize();
        double force_base[6]={0,0,0,0,0,0};
        double delta_x[3] = {0,0,0};
        double delta_rpy[3] = {0,0,0};

        sub_ur_update();
        KDL::Frame target_frame = frame_wrist3_base_;
        KDL::Frame start_frame = frame_wrist3_base_;
        KDL::JntArray target_jnt(6);
        uint64_t sec0 = ros::Time::now().toSec();
        ROS_INFO_STREAM("force  interaction is ok" );




        double dis_fruit_z = 0.10;  //手末端离水果距离
        double pos_pre = 0.001;  //位置精度5mm

        robotiq_hand_move(80,5,10);

        ROS_INFO_STREAM("move to grasp fruit ");

        fruit[2] += dis_fruit_z + hand_bias_z;
        move_line_fix_end(fruit,0.2);
        ros::Duration(0.2).sleep();
//        z下降一定距离 抓水果
        fruit[2] -= dis_fruit_z + 0.012;
        move_line_fix_end(fruit,0.2);
        ros::Duration(0.2).sleep();
        robotiq_hand_move(12,5,10);
        ros::Duration(6).sleep();

//        带着水果上升一定距离
        fruit[2] += dis_fruit_z;
        move_line_fix_end(fruit,0.2);
        ros::Duration(0.2).sleep();

        ROS_INFO_STREAM("move to hand the fruit ");


//        获取手上marker位置 z 轴最小的 没有实时规划
        KDL::Vector hand_marker_const;    //此时手上marker 位置
        while( !get_marker_hand(hand_marker_const) )
        {
            ROS_INFO_STREAM("try to get marker on hand...");
            ros::Duration(0.5).sleep();
        }

        hand_marker_const[2] += dis_fruit_z + hand_bias_z + 0.02;
//        运动到手上方
        move_line_fix_end(hand_marker_const,0.2);
        KDL::Vector hand_move;  //手移动了一定距离
        if( !get_marker_hand(hand_move) ){
            ROS_WARN("cann't get marker on hand ");
            return -2; }
        hand_move[2] += dis_fruit_z + hand_bias_z + 0.02;
        move_line_fix_end(hand_move,0.2);

//        ros::Duration(1).sleep();

        KDL::Vector marker1(0,0,10);
        end_point_filter_.initialize();
        sub_ur_update();
        KDL::Frame next_goal_frame = frame_wrist3_base_;
        KDL::JntArray next_joints(6);
        ros::Rate loop_rate(500);

        double sum_force_up = 50;
        double sum_force_low = 10;
        while( ros::ok() && (! myo_grasp || (marker1 - frame_wrist3_base_.p).Norm() > pos_pre ) )
        {
            ros::spinOnce();
            double sum_force = wrench_base_.force.Norm();

            // contact force protection
            if (sum_force>sum_force_up){
                ROS_ERROR("contact force is too big. stop moving");
                break;
            }
            else if(sum_force < sum_force_up && sum_force > sum_force_low)
            {
                for (int i = 0; i <3 ; ++i) {
                    force_base[i]   = wrench_base_.force.data[i];
                    force_base[i+3] = wrench_base_.torque.data[i];
                }

                target_frame.p = frame_wrist3_base_ .p;    // consider xyz without rpy rotation.
                double time_now_d = ros::Time::now().toSec() - sec0;
                controller1.step(force_base,delta_x,delta_rpy);

                for (int j = 0; j < 3; ++j) {
                    if (abs(wrench_base_.force.data[j])<5) {
                        delta_x[j] = 0;
//                    controller1.initialize();
                    }
                    target_frame.p.data[j] +=delta_x[j]/1000.0;    // mm to m
                }
                ROS_INFO_STREAM("delta_xyz: "<< delta_x[0]<<" "<< delta_x[1]<<" "<< delta_x[2]<<" "
                                             << wrench_base_.force.data[0]<<" "<< wrench_base_.force.data[1]
                                             <<" "<<wrench_base_.force.data[2]<<" ");
                move_line_dynamic(target_frame.p,0.1);
            }
            else {
//            获取手上的点，此时抓取物在手上方 选取z小的
                if( !get_marker_hand(marker1) ){
                    ROS_FATAL_STREAM("cann't get marker on hand in real time motion plan ");
                    return -2; }

                if(marker1.y() < -0.7)
                {
                    ROS_FATAL_STREAM("the marker y is over the edge of table " << marker1.y());
                    return -1;
                }
                marker1[2] += dis_fruit_z + hand_bias_z + 0.02;   //distance between hand and marker z
                //            ROS_INFO_STREAM("now dist "<<(marker1 - frame_wrist3_base_.p).Norm() - pos_pre);
                ros::spinOnce();
                if( (marker1 - frame_wrist3_base_.p).Norm() > pos_pre ) {
                    move_line_dynamic(marker1,0.2);
                }
            }
            loop_rate.sleep();
        }

        ros::Duration(3).sleep();
        robotiq_hand_move(80,10,200);
        return 1;
    }


    bool get_marker_hand(KDL::Vector &marker_on_hand){
        KDL::Vector hand_marker(0,0,10);
        ros::spinOnce();
        std::vector<KDL::Vector> markers = first_body_markers_;
//            获取手上的点，此时抓取物在手上方 选取z小的
        for (unsigned int i = 0; i < markers.size(); i++) {
            while (markers[i].x() > 10) {
                bmotion_capture_sub_ = false;
                do {
                    ros::spinOnce();
                } while (!bmotion_capture_sub_);
                markers[i] = first_body_markers_[i];
            }
            if (  !( markers[i].y() > 0 && markers[i].x() < -0.4  ) && markers[i].z() < hand_marker.z() ) {
                hand_marker = markers[i];
            }
        }
        if(hand_marker.y() > -0.7 && hand_marker.y() < 2)
        {
            marker_on_hand = hand_marker;
            return true;
        } else return false;
    }

    void test_track_marker(){
        ros::Rate loop_rate(500);
        KDL::Vector marker1(0,0,0);
//        uint64_t nsec0 = ros::Time::now().toNSec();
        do
        {
//            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
//            double time_now_d = time_now / 1e9;

            int seq = 0;   //第几个marker
            if(first_body_markers_.size() == 0 )
            {
                ROS_INFO_STREAM("there is no marker "<<first_body_markers_.size() );
                continue;
            }
            marker1 = first_body_markers_[seq];
//            ROS_INFO_STREAM("marker "<<marker1.x()<<" "<<marker1.y()<<" "<<marker1.z() );
//            ROS_INFO_STREAM("markers number "<<first_body_markers_.size());
            while (marker1[0]>10){
                    bmotion_capture_sub_ = false;
                    do{
                        ros::spinOnce();
                    }while(!bmotion_capture_sub_);

                marker1 = first_body_markers_[seq];
//                ROS_INFO_STREAM("new tracking marker "<< marker1[0]<<" "<<marker1[1]<<" "<<marker1[2]);
            }

            ROS_INFO_STREAM("tracking marker "<< marker1[0]<<" "<<marker1[1]<<" "<<marker1[2]);

            move_line(marker1,5);

//            //record
//            std::ofstream file3;
//            std::ostringstream oss;
//            int name_suffix = 1;
//            oss << name_suffix;
//            std::string filename = oss.str() + "_servoj_tracking_real_robot";
//            std::string file = "/home/zd/project/catch_bottle/catch_control/record_servoj_sim/" + filename + ".txt";
//            file3.open(file.c_str(), std::ios::app);
//            file3 << time_now_d << " ";
//            KDL::Frame tool_frame = frame_wrist3_base_;
//            double rpy[3];
//            tool_frame.M.GetRPY(rpy[0],rpy[1],rpy[2]);
//            file3 << tool_frame.p.data[0]<< " " <<tool_frame.p.data[1] << " " << tool_frame.p.data[2]<< " "
//                  << rpy[0]<< " " << rpy[1] << " " <<rpy[2];
//            file3 << std::endl;
//            file3.close();
//            loop_rate.sleep();
//            if (time_now_d>20)
//                break;

            loop_rate.sleep();
        }while( (marker1 - frame_wrist3_base_.p).Norm() > 0.005) ;

    }

    bool choose_fruit(KDL::Vector &choosed_fruit) {
        force_update_all_state();
        bool get_fruit_position = false;
        std::vector<KDL::Vector> body_markers = first_body_markers_;
        do {
            bmotion_capture_sub_ = false;
            ros::spinOnce();
            body_markers = first_body_markers_;
        } while (!bmotion_capture_sub_ || (body_markers.size() <= 2));
//        判断是物体的marker还是手上的
        std::vector<KDL::Vector> fruit_markers,hand_markers;
        for(unsigned int i=0;i<body_markers.size();i++)
        {
            while (body_markers[i].data[0]>10){
                force_update_all_state();
                body_markers[i] = first_body_markers_[i];
            }
            if(body_markers[i][1] > 0 )            //粗略分割
            {
                fruit_markers.push_back(body_markers[i]);
                ROS_INFO_STREAM("fruit markers "<< body_markers[i].data[0]<<" "<<body_markers[i].data[1]<<" "<<body_markers[i].data[2]);
            }
            else{
                hand_markers.push_back(body_markers[i]);
                ROS_INFO_STREAM("hand markers "<< body_markers[i][0]<<" "<<body_markers[i][1]<<" "<<body_markers[i][2]);
            }
        }



        Eigen::Vector3d marker1(hand_markers[0][0], hand_markers[0][1], hand_markers[0][2]);
        Eigen::Vector3d marker2(hand_markers[1][0], hand_markers[1][1], hand_markers[1][2]);


        double length_point_line = 0.1;
        for (unsigned int i = 0; i < fruit_markers.size(); i++) {

            Eigen::Vector3d fruit_vec(fruit_markers[i][0], fruit_markers[i][1], fruit_markers[i][2]);

            Eigen::Vector3d cross_line = (fruit_vec-marker1).cross(marker2 - marker1);
            double length_point_line_tem = cross_line.norm() /(marker2-marker1).norm();
            if (length_point_line_tem < length_point_line) {
                length_point_line = length_point_line_tem;
                choosed_fruit = KDL::Vector(fruit_vec[0],fruit_vec[1],fruit_vec[2]);
                get_fruit_position = true;
            }
        }
        return get_fruit_position;
    }

    void test_free_drive(){
        std_msgs::String temp;
        std::string cmd_str;
        cmd_str = "def myProg():\n";
        cmd_str += "\twhile (True):\n";
        cmd_str += "\t\tfreedrive_mode()\n";
        cmd_str +="\t\tsync()\n";
        cmd_str += "\tend\n";
        cmd_str +="end\n";

//        cmd_str ="def move_forward_10cm():\n";
//        cmd_str +="\tset_digital_out(1,True)\n";
//        cmd_str +="end\n";

//        can move without stop pos_based_controller_manager
//        cmd_str ="def move_forward_10cm():\n";
//        cmd_str +="\tmovej([1.57,-1.57,-1.57,3.14,-1.57,1.57], a=1.4, v=1.05, t=0, r=0)\n";
//        cmd_str +="end\n";
        temp.data = cmd_str;
        pub_free_drive_.publish(temp);
    }

    void test_move_line(KDL::Vector end_point)
    {
//        ros::AsyncSpinner spinner(6);
//        spinner.start();

        KDL::Rotation grasp_rota;
        grasp_rota = grasp_rota.RotY(M_PI);  // 抓取姿态


        KDL::Frame end_frame_compu;
        uint64_t nsec0 = ros::Time::now().toNSec();

        ros::Rate loop_rate(500);
        force_update_all_state();
        KDL::Frame start_frame = frame_wrist3_base_;
        while ( (frame_wrist3_base_.p - end_point).Norm() > 0.005 ){
            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
            double time_now_d = time_now / 1e9;
            ros::spinOnce();
            move_line(end_point,5,start_frame, end_frame_compu);

//            if ( (frame_wrist3_base_.p - end_point_).Norm()> 0.08 )
//                ROS_WARN("bug");
            //record
            std::ofstream file3;
            std::ostringstream oss;
            int name_suffix = 9;
            oss << name_suffix;
            std::string filename = oss.str() + "_servoj_tracking_real_robot";
            std::string file = "/home/zd/project/catch_bottle/catch_control/record_servoj_sim/" + filename + ".txt";
            file3.open(file.c_str(), std::ios::app);
            file3 << time_now_d << " ";
            file3 << end_frame_compu.p.data[0]<< " " <<end_frame_compu.p.data[1] << " " << end_frame_compu.p.data[2]<< " "
                  << end_point_.data[0]<< " " << end_point_.data[1] << " " <<end_point_.data[2];
            file3 << std::endl;
            file3.close();
            loop_rate.sleep();
            if (time_now_d>20)
                break;
        }


    }

//    计算逆矩阵 输出
    void compu_inverse_frame(KDL::Frame frame_input){
//        KDL::Frame base2map;
//        base2map.M = KDL::Rotation::Quaternion(-0.0001191, -0.000309095, -0.70524, 0.708969);
//        base2map.p = KDL::Vector(0.376272, -0.321799, 0.0396726);
        KDL::Frame map2base = frame_input.Inverse();
        double quat[4];
        map2base.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
        ROS_INFO_STREAM("map2base p"<<map2base.p.x()<<" "<<map2base.p.y()<<" "<<map2base.p.z() );
        ROS_INFO_STREAM("map2base M"<<quat[0]<<" "<<quat[1]<<" "<<quat[2] );
    }

    void sub_ur_update()
    {
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while (!bur_sub_);
    }

    bool ur_wrench_ok()
    {
        double delta_wrench = (ur_wrench_now - start_wrench).Norm();
        for(int i=0;i<3;i++)
        {
            if( delta_wrench > 20)
            {
                ROS_WARN_STREAM("ur_wrench is to big "<< delta_wrench);
                return false;
            }
            ROS_INFO_STREAM("deta_ur_wrench "<<delta_wrench);
        }
        return true;
    }


private:

    void subMotionCaptureCB(visualization_msgs::MarkerArray state) {
//        ROS_INFO_STREAM("size"<<state.markers.size());
        first_body_markers_.clear();
        std::vector<KDL::Vector> first_body_markers_in_nokov ;

        std::vector<KDL::Vector> hole_markers ;
        std::vector<geometry_msgs::Point> markers_position;

        for (unsigned int i = 0; i < state.markers.size(); ++i) {
            markers_position.push_back(state.markers.at(i).pose.position);
            std::string hole_tmp = "u";  // first body name
            if (state.markers.at(i).ns.substr(0,1)==hole_tmp) {
                KDL::Vector m_tmp  = KDL::Vector(markers_position[i].x,markers_position[i].y,markers_position[i].z);
                hole_markers.push_back(m_tmp);
                first_body_markers_in_nokov.push_back(m_tmp);
            }
        }
//        ROS_INFO_STREAM("marker size"<<first_body_markers_in_nokov.size());

        KDL::Frame T_b2m;
        T_b2m.p = KDL::Vector(0.376272, -0.321799, 0.0396726);
        T_b2m.M = KDL::Rotation::Quaternion(-0.0001191, -0.000309095, -0.70524, 0.708969);
        for (unsigned int i = 0; i < first_body_markers_in_nokov.size(); ++i) {
            KDL::Vector tmp1 = T_b2m.Inverse() * first_body_markers_in_nokov[i];   // 点在bask_link下的坐标
            first_body_markers_.push_back(tmp1);
            if(first_body_markers_[i][1] < 0 && first_body_markers_[i][1] < 10 )
                follow_marker_seq = i;
        }

        bmotion_capture_sub_ = true;
    }

    void subHandStatesCB(robotiq_85_msgs::GripperStat state){
        hand_state_ = state;
        b_hand_sub_ = true;
    }

    void subJointStatesCB(sensor_msgs::JointState state)
    {
        KDL::JntArray jntArr;
        KDL::JntArray jntSpeed;
        KDL::JntArray jntCur;
        jntArr.resize(6);
        jntSpeed.resize(6);
        jntCur.resize(6);
        int n = state.name.size();
        for (int i = 0; i < 6; ++i)//joint_names_
        {
            int x = 0;
            for (; x < n; ++x)//state
            {
                if (state.name[x] == ( joint_names_[i])) {
                    jntArr(i) = state.position[x];
                    jntSpeed(i) = state.velocity[x];
                    jntCur(i) = state.effort[x];
                    break;
                }
            }

            if (x == n) {
                ROS_ERROR_STREAM("Error,  joint name : " << joint_names_[i] << " , not found.  ");
                return;
            }
        }
        current_JntArr_ = jntArr;
//      current_JntSpeed = jntSpeed;
//      current_JntCur_ = jntCur;

        //fk
        p_fk_solver_->JntToCart(current_JntArr_, frame_wrist3_base_);    //frame_wrist3_base_ 为正运动学计算出的位姿
        end_point_.data[0] = frame_wrist3_base_.p.data[0];
        end_point_.data[1] = frame_wrist3_base_.p.data[1];
        end_point_.data[2] = frame_wrist3_base_.p.data[2];
        geometry_msgs::Point end_point;
        end_point.x = end_point_.x();
        end_point.y = end_point_.y();
        end_point.z = end_point_.z();
        pub_end_point_.publish(end_point);
//        if ( (frame_wrist3_base_.p - end_point_).Norm()> 0.08 )
//            ROS_WARN("bug");
        bur_sub_ = true;
    }

    void sub_myo_handCall(ros_myo::EmgArray hand_state) {
        ros::Rate loop_rate(1);
        ros_myo::EmgArray my_hand_state = hand_state;
        robotiq_85_msgs::GripperCmd hand_cmd;
        hand_cmd.speed = 5.0/1000;
        hand_cmd.force = 50.0/1000;
        double sum_arr = 0;
        for(unsigned int i=0;i<my_hand_state.data.size();i++)
            sum_arr += my_hand_state.data[i];
        if(sum_arr/my_hand_state.data.size() > 300)
            myo_grasp = true;
        else
            myo_grasp = false;
        ROS_INFO_STREAM("myo_state "<<myo_grasp);

//    ROS_INFO_STREAM("grasp or not "<<myo_grasp);
//    loop_rate.sleep();
    }


    // UR5e base 相对于base坐标系，base_link绕z旋转180度得base坐标系
    void sub_wrench(geometry_msgs::WrenchStamped state){
        wrench_base_.force =  KDL::Vector(state.wrench.force.x,state.wrench.force.y,state.wrench.force.z );
        wrench_base_.torque =  KDL::Vector(state.wrench.torque.x,state.wrench.torque.y,state.wrench.torque.z );
        KDL::Rotation R_base2base_link = KDL::Rotation::RPY(0,0,M_PI);
        wrench_base_.force = R_base2base_link * wrench_base_.force;
        wrench_base_.torque = R_base2base_link * wrench_base_.torque;
        wrench_base_ = wrench_base_ - wrench_repair_;
//        ROS_INFO_STREAM("wrench_repaired: "<<wrench_base_.force.data[0]<<" "
//                                           <<wrench_base_.force.data[1]<<" "
//                                           <<wrench_base_.force.data[2]<<" ");
//        geometry_msgs::WrenchStamped ur_wrench = state;
//        geometry_msgs::Vector3 ur_wrench_tem = ur_wrench.wrench.force;
//        ur_wrench_now = KDL::Vector(ur_wrench_tem.x, ur_wrench_tem.y, ur_wrench_tem.z );
        bsub_wrench_ = true;
//        if( (ur_wrench_now - start_wrench).Norm() > 20 )
//        {
//            wrench_ok = false;
//            ROS_FATAL_STREAM("the wrench is to big "<< (ur_wrench_now - start_wrench).Norm() );
//        } else wrench_ok = true;
//        ROS_INFO_STREAM("wrench "<< (ur_wrench_now - start_wrench).Norm() <<" wrench now  "<<ur_wrench_now.z());
    }

private:
    ros::NodeHandle nh_;
    TRAC_IK::TRAC_IK *p_tracik_solver_;
    KDL::ChainFkSolverPos_recursive *p_fk_solver_;
    ros::Subscriber sub_;   //UR5e sub
    KDL::JntArray current_JntArr_;
    Client *client_;
    Client *client_servoj_;
    bool bsub_;
    bool bur_sub_;
    std::vector<std::string> joint_names_;
    KDL::Frame frame_wrist3_base_;//fk result
    KDL::Vector end_point_;

    std::vector<double> all_angle;

    // Robotiq Hand
    ros::Subscriber sub_hand_;
    ros::Publisher pub_hand_cmd_;

//    UR5
    ros::Publisher pub_free_drive_;
    ros::Publisher pub_end_point_;
    ros::Subscriber ur_wrench_;
    KDL::Vector start_wrench;
    bool bsub_wrench_;
    KDL::Vector ur_wrench_now;
    bool wrench_ok = true;

    //Motion capture
    ros::Subscriber sub_markers_;
    bool bmotion_capture_sub_;
    bool bstart_cal_;
    std::vector<KDL::Vector> first_body_markers_ ;
    int follow_marker_seq = -1;
    bool record_begin = false;


    //Hand of Lei
    ros::Publisher hand_cmd;

    robotiq_85_msgs::GripperStat hand_state_;
    bool b_hand_sub_;

//    软体手 距离 tool 偏差   可以再加一个作为调整偏移
    double hand_bias_z = 0.146 + 0.077;

//    myo
    ros::Subscriber sub_myo_hand_;
    bool myo_grasp = false;

    // simulink2C
    end_poing_filter end_point_filter_;


    // wrench
    KDL::Wrench wrench_base_;
    KDL::Wrench wrench_repair_;


};








#endif //PROJECT_CATCH_BOTTLE_H

//
// Created by zd on 19-6-1.
//

#ifndef ros_myo_HUMAN_INTERACT_DEMO_H
#define ros_myo_HUMAN_INTERACT_DEMO_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "robotiq_85_msgs/GripperCmd.h"
#include "robotiq_85_msgs/GripperStat.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <trac_ik/trac_ik.hpp>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <ros_myo/EmgArray.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class UR_gripper{

public:
    UR_gripper(ros::NodeHandle _nh,std::string PLANNING_GROUP){

        kinect2base_link.p = KDL::Vector(-1.06128558384, 0.067565534107, 0.717098767451);
        kinect2base_link.M = KDL::Rotation::Quaternion(-0.649109588692, 0.638784020982, -0.297812759475, 0.286215437566);

        base_link2map.p = KDL::Vector(0.376272, -0.321799, 0.0396726);
        base_link2map.M = KDL::Rotation::Quaternion(-0.0001191, -0.000309095, -0.70524, 0.708969);

        base2ur_world.p = KDL::Vector(-0.01 ,0.377 ,0.808);
        base2ur_world.M = KDL::Rotation::RPY(0,0,1.570796327);

        move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        sub_hand_state = _nh.subscribe("/gripper/stat",1 , &UR_gripper::subHandStatesCB,this);
        sub_camera_rec = _nh.subscribe("/recognized_object_array",1 , &UR_gripper::subCamera,this);
        sub_camera_rgb = _nh.subscribe("/visualization_marker",1,&UR_gripper::subCameraRgb,this);
        sub_myo_hand_ = _nh.subscribe("/myo_raw/myo_emg",1,&UR_gripper::sub_myo_handCall,this);
        pub_hand_cmd_ = _nh.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd",1);

        sub_markers_ = _nh.subscribe("/vis_markers", 1, &UR_gripper::subMotionCaptureCB,this);

        move_group->allowReplanning(true);
        move_group->setGoalPositionTolerance(0.001);
        move_group->setGoalOrientationTolerance(0.01);
        move_group->setMaxVelocityScalingFactor(0.1);
        move_group->setMaxAccelerationScalingFactor(0.1);
        ROS_INFO_STREAM("planning frame: "<< move_group->getPlanningFrame(); );

        client_servoj_ = new Client("/UR5e/pos_based_pos_traj_controller/follow_joint_trajectory",true);
        ROS_INFO("wait for servoj client");
        client_servoj_->waitForServer(ros::Duration());
        ROS_INFO("servoj clent connect!");

    }
    ~UR_gripper(){ delete move_group; }

    void subCamera(const object_recognition_msgs::RecognizedObjectArray& box_obj);

    void subCameraRgb(visualization_msgs::Marker marker);

    void subMotionCaptureCB(visualization_msgs::MarkerArray state);

    void sub_myo_handCall(ros_myo::EmgArray hand_state);

    void com_plan_CartsianPath(double point[], int number_point);

    void robotiq_hand_move(float position,float vel, float force);

    bool move_to_start_position(std::vector<double> joint_start_position);

    bool interpolate_line(std::vector< KDL::Vector > points, double time);

    bool catch_rgb_fruit(double wait_time);

    bool catch_rgb_fruit(std::vector<double> fruit);

    void servoj_moveto(KDL::JntArray target, double time,bool wait_for_D);

    void test_move_line(KDL::Vector end_point);


    bool catch_pose_box(double wait_time);

    void subHandStatesCB(robotiq_85_msgs::GripperStat state){
        hand_state_ = state;
        finish_hand_sub_ = true;
    }

    Eigen::Vector3d choose_fruit();

    Eigen::Vector3d calculate_center_point (std::vector<KDL::Vector> markers );

    bool huaman_interact();

    void force_update_all_state(){
        finish_hand_sub_ = false;
        sub_motion_capture_ = false;
        do{
            ros::spinOnce();
        }while(!finish_hand_sub_ || !sub_motion_capture_);
    }

    void add_box(moveit_msgs::CollisionObject& collision_object,double bottle_height[],
                 double bottle_x_y_z[], KDL::Frame grasp_frame, std::string object_name ,std::string reference_frame);

public:
//    KDL::Vector box_obj_position;
//    Eigen::Vector3d box_axis_z, grasp_point;
    KDL::Frame grasp_frame_bef;
    std::vector<KDL::Frame> grasp_frame_bef_vec;

    KDL::Frame kinect2base_link;
    KDL::Frame base2ur_world;
    KDL::Frame base_link2map;

    moveit::planning_interface::MoveGroupInterface* move_group;
    bool get_box_frame = false;
    bool get_camera_rgb = false;
    double camera_box_position_rgb[3] = { 0.0, 0.0, 0.0};
    std::vector<double> box_rgb_vec;

//    myo 控制手抓开合状态
    bool myo_grasp = false;

    double point_up_collectBox[3] = { 0.44, 0.03, 1.1 };   //盘子上方位置
    double softhand = 0.125;    //软体手指中心到tool原点距离

//    动态捕捉系统
    std::vector<KDL::Vector> first_body_markers_;
    Eigen::Vector3d center_point_marker;
    bool sub_motion_capture_;

//    ros_control server
    typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;
    Client *client_servoj_;

    control_msgs::FollowJointTrajectoryGoal my_goal;


private:

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Subscriber sub_hand_state;
    ros::Subscriber sub_camera_rec;
    ros::Subscriber sub_camera_rgb;
    ros::Subscriber sub_markers_;
    ros::Subscriber sub_myo_hand_;

    ros::Publisher pub_hand_cmd_;
    robotiq_85_msgs::GripperStat hand_state_;

    bool finish_hand_sub_;
    tf::TransformBroadcaster br;

};

void UR_gripper::subCamera(const object_recognition_msgs::RecognizedObjectArray& box_obj) {
    grasp_frame_bef_vec.clear();
    for(unsigned int i=0;i < box_obj.objects.size();i++ ) {

        object_recognition_msgs::RecognizedObject obj_rec0 = box_obj.objects[i];
        geometry_msgs::Pose pose = obj_rec0.pose.pose.pose;
        KDL::Frame box_frame, grasp_frame;
        KDL::Vector z_before(0, 0, 1), box_orientation_z;

        box_frame.p = KDL::Vector(pose.position.x, pose.position.y, pose.position.z);
        box_frame.M = KDL::Rotation::Quaternion(pose.orientation.x,
                                                pose.orientation.y, pose.orientation.z, pose.orientation.w);

//        ROS_INFO_STREAM("box_frame M "<<box_frame.M.data[0]<<" "<<box_frame.M.data[1]<<" "<<box_frame.M.data[2]<<std::endl);
//        ROS_INFO_STREAM("box_frame M "<<box_frame.M.data[3]<<" "<<box_frame.M.data[4]<<" "<<box_frame.M.data[5]<<std::endl);
//        ROS_INFO_STREAM("box_frame M "<<box_frame.M.data[6]<<" "<<box_frame.M.data[7]<<" "<<box_frame.M.data[8]<<std::endl);

        box_frame = kinect2base_link * box_frame;   //相对与base_link

        box_orientation_z = box_frame.M.UnitZ();

        if (box_orientation_z[2] < 0) {box_orientation_z = -box_orientation_z;}  //保证指向上
        Eigen::Vector3d grasp_axis_z_before, global_z(0, 0, 1), grasp_axis_z, grasp_axis_x,
                grasp_axis_y(box_orientation_z.x(), box_orientation_z.y(), box_orientation_z.z());

        double rad_glZ_boxZ = dot(box_orientation_z, z_before);  //瓶子z轴和global_z 夹角 cos
//        水平极端位姿判断
        if (rad_glZ_boxZ < 0.05) {
            grasp_axis_z = -global_z;
            grasp_axis_y[2] = 0;
            grasp_axis_y[0] = sqrt(1 - pow(grasp_axis_y[1], 2));  //可交换
        } else if (rad_glZ_boxZ > 0.95) {
            double length_box_base = sqrt(pow(box_frame.p.x(), 2) + pow(box_frame.p.y(), 2));
            grasp_axis_z = Eigen::Vector3d(box_frame.p.x() / length_box_base, box_frame.p.y() / length_box_base, 0.0);
            grasp_axis_y[0] = 0;
            grasp_axis_y[1] = 0;
            grasp_axis_y[2] = 1;
        } else {
            grasp_axis_z_before = grasp_axis_y.dot(grasp_axis_y) / (grasp_axis_y.dot(
                    global_z)) * global_z - grasp_axis_y;
            grasp_axis_z = -grasp_axis_z_before / sqrt(grasp_axis_z_before.dot(grasp_axis_z_before));
        }

        grasp_axis_x = grasp_axis_y.cross(grasp_axis_z);

        grasp_frame.p = KDL::Vector(box_frame.p.x(), box_frame.p.y(), box_frame.p.z());
        grasp_frame.M.UnitX(KDL::Vector(grasp_axis_x[0], grasp_axis_x[1], grasp_axis_x[2]));
        grasp_frame.M.UnitY(KDL::Vector(grasp_axis_y[0], grasp_axis_y[1], grasp_axis_y[2]));
        grasp_frame.M.UnitZ(KDL::Vector(grasp_axis_z[0], grasp_axis_z[1], grasp_axis_z[2]));
        grasp_frame = base2ur_world * grasp_frame;
        grasp_frame_bef = grasp_frame;

        double quat[] = {0, 0, 0, 1};
        grasp_frame.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);

//        在rviz中发布坐标系
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(grasp_frame.p.x(), grasp_frame.p.y(), grasp_frame.p.z()));
        tf::Quaternion q;
        q.setX(quat[0]);
        q.setY(quat[1]);
        q.setZ(quat[2]);
        q.setW(quat[3]);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                              "ur_world", "bottle"+std::to_string(i) ));

        grasp_frame_bef_vec.push_back(grasp_frame);
        get_box_frame = true;
    }


}

void UR_gripper::subCameraRgb(visualization_msgs::Marker marker) {
    std::vector<double> rgb_box_pos_vec;
    std::vector<Eigen::Vector3d> rgb_box_pos_vec_tem;

    for(unsigned int i=0;i<marker.points.size();i++){
        //        get_camera_rgb = false;
        geometry_msgs::Point box_rgb = marker.points[i];
        if(box_rgb.x < -0.29 && box_rgb.x > -0.75 && box_rgb.y < 0.35 && box_rgb.y > -0.11)
        {
            KDL::Vector pos_vec(box_rgb.x ,box_rgb.y, box_rgb.z);
//    数据已经相对于base_link
            KDL::Vector rgb_box_base = base2ur_world * pos_vec;
//        可能有无效点
            if(rgb_box_base.x() < 1){
                rgb_box_pos_vec.push_back(rgb_box_base.x() );
                rgb_box_pos_vec.push_back(rgb_box_base.y() );
//                rgb_box_pos_vec.push_back(0.19);
                rgb_box_pos_vec.push_back(rgb_box_base.z() );

                camera_box_position_rgb[0] = rgb_box_base.x();
                camera_box_position_rgb[1] = rgb_box_base.y();
                camera_box_position_rgb[2] = rgb_box_base.z();

                get_camera_rgb = true;
            }
        }

    }
    box_rgb_vec = rgb_box_pos_vec;
}

void UR_gripper::subMotionCaptureCB(visualization_msgs::MarkerArray state) {

    first_body_markers_.clear();
    std::vector<geometry_msgs::Point> markers_position;
    std::string hole_tmp = "u1";  // first body name
    for (unsigned int i = 0; i < state.markers.size(); ++i) {
        markers_position.push_back(state.markers.at(i).pose.position);
        if (state.markers.at(i).ns.substr(0,2)!=hole_tmp) {
            KDL::Vector m_tmp(markers_position[i].x,markers_position[i].y,markers_position[i].z);
            first_body_markers_.push_back(m_tmp);
        }
    }
    sub_motion_capture_ = true;
}

void UR_gripper::sub_myo_handCall(ros_myo::EmgArray hand_state) {
    ros::Rate loop_rate(1);
    ros_myo::EmgArray my_hand_state = hand_state;
    robotiq_85_msgs::GripperCmd hand_cmd;
    hand_cmd.speed = 5.0/1000;
    hand_cmd.force = 50.0/1000;
    if(my_hand_state.data.at(0) > 300)
    {
        myo_grasp = true;
        hand_cmd.position = 20.0/1000;   // scale 0-0.085mm30
        pub_hand_cmd_.publish(hand_cmd);
    }
    else {
        myo_grasp = false;
        hand_cmd.position = 85.0/1000;
        pub_hand_cmd_.publish(hand_cmd);
    }
    ROS_INFO_STREAM("grasp or not "<<myo_grasp);
    loop_rate.sleep();
}

void UR_gripper::robotiq_hand_move(float position,float vel, float force){
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

void UR_gripper::com_plan_CartsianPath(double *point,int number_point) {
    geometry_msgs::Pose target_pose = move_group->getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    for(int i=0; i<number_point * 4; i += 4)
    {
//        绝对和相对
        if(point[i+3])
        {
            target_pose.position.x = point[i];
            target_pose.position.y = point[i+1];
            target_pose.position.z = point[i+2];
        } else{
            target_pose.position.x += point[i];
            target_pose.position.y += point[i+1];
            target_pose.position.z += point[i+2];
        }
        waypoints.push_back(target_pose);
    }


    move_group->setMaxAccelerationScalingFactor(0.1);
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

//        servoj ros_control
//        my_goal.trajectory.points = trajectory.joint_trajectory.points;
//        my_goal.trajectory.header.stamp = ros::Time::now();
//        my_goal.trajectory.joint_names = trajectory.joint_trajectory.joint_names;
//        client_servoj_->sendGoal(my_goal);

//        ros::Duration(3).sleep();
    }

    ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

}

bool UR_gripper::catch_rgb_fruit(double wait_time) {
    std::vector<double> rgb_box_vec;
    ros::Time start = ros::Time::now();
//    获取水果位置
    do{
        get_camera_rgb = false;
        do{
            ros::spinOnce();
        }while(!get_camera_rgb);
        ros::Duration(1).sleep();
        rgb_box_vec = box_rgb_vec;
        ROS_INFO_STREAM("point size  "<< rgb_box_vec.size()/3);
        if(ros::Time::now() - start > ros::Duration(wait_time) )
        {
            ROS_INFO_STREAM("There isn't any fruit to grasp ");
            return false;
        }
    }while(!(box_rgb_vec.size()/3) );


//    抓取 彩色物体
    for(unsigned int i=0;i < rgb_box_vec.size();i += 3)
    {

        double rgb_bias_x_y[] = { 0.0, 0.0, 0.0};

        double point_up_bottle_plan_rgb[] = {
                0,0,0.15,0,
                rgb_box_vec[i], rgb_box_vec[i+1], rgb_box_vec[i+2] + 0.15 + softhand,  1,
                0.0, 0.02, -0.15 + softhand - 0.035 + rgb_bias_x_y[2],0 };

        com_plan_CartsianPath(point_up_bottle_plan_rgb,3);

        robotiq_hand_move(45,5,50);
        ros::Duration(4).sleep();
//        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pick the fruit ");

        double back_point_plan[] = {0,0,0.15,0, point_up_collectBox[0],point_up_collectBox[1],point_up_collectBox[2],1};
        com_plan_CartsianPath(back_point_plan,2);

        robotiq_hand_move(85,5,50);

        ros::Duration(3).sleep();
    }
    return true;
}

bool UR_gripper::catch_rgb_fruit(std::vector<double> fruit) {
//    起始点到目标物体 抓取
    double go_grasp[] = {
            0,0,0.15,0,
            fruit[0], fruit[1], fruit[2] + 0.15 + softhand,  1,
            0.0, 0.0, -0.15 + softhand,0 };

    com_plan_CartsianPath(go_grasp,3);

//    回到手上方
//    Eigen::Vector3d center_point;
//    center_point = calculate_center_point (first_body_markers_);
//    double back_point_plan[] = {0,0,0.15,0, center_point[0],center_point[1],center_point[2],1};
//    com_plan_CartsianPath(back_point_plan,2);
//
//    double goal_distance_2 = 10.0;
//    do{
//        center_point = calculate_center_point (first_body_markers_);
//        geometry_msgs::Point cur_pos_point = move_group->getCurrentPose().pose.position;
//        goal_distance_2 = pow(fruit[0] - cur_pos_point.x,2) + pow(fruit[1] - cur_pos_point.y,2) +
//                pow(fruit[2] - cur_pos_point.z,2);
//
//    }while(sqrt(goal_distance_2) < 0.05);

    robotiq_hand_move(45,5,50);

    return true;
}

//    测试直线性能
void UR_gripper::test_move_line(KDL::Vector end_point) {
    geometry_msgs::Point pos_now = move_group->getCurrentPose().pose.position;
    KDL::Vector pos_now_vec(pos_now.x, pos_now.y, pos_now.z);
    ros::Rate loop_rate(500);
    do{
        double end_point_com[] = { end_point.x(), end_point.y(), end_point.z()};
        com_plan_CartsianPath(end_point_com,1);
        ROS_INFO_STREAM("hello move test ");
        loop_rate.sleep();
    }while ( (pos_now_vec - end_point).Norm() > 0.005 );
}

bool UR_gripper::catch_pose_box(double wait_time) {
//    抓取需要调整抓取姿态的物体
    ros::Time start = ros::Time::now();
    do{
        ros::spinOnce();
        if(ros::Time::now() - start > ros::Duration(wait_time) )
        {
            ROS_INFO_STREAM("There isn't any (pose)box to grasp ");
            return false;
        }
    }while( !get_box_frame);

    std::vector<KDL::Frame> grasp_frame_ber_vec = grasp_frame_bef_vec;

//    抓取
    for(unsigned int i=0;i<grasp_frame_ber_vec.size();i++)
    {
        KDL::Frame grasp_frame_urWorld = grasp_frame_ber_vec[i];
        //    根据瓶子的方向旋转一定角度 初始位置已经在joint[5] 中心附近

        std::vector<double> joint_value = move_group->getCurrentJointValues();
        geometry_msgs::Pose pose_now = move_group->getCurrentPose().pose;
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

        move_group->setJointValueTarget(joint_value);
        move_group->move();

        ros::Duration(2).sleep();

//        位姿抓取
        double point_up_bottle_plan[] ={
                0,0,0.15,0,
                grasp_frame_urWorld.p.x(), grasp_frame_urWorld.p.y(), grasp_frame_urWorld.p.z() + 0.15 + softhand,  1,
                0,-0.02,-0.15 + softhand - 0.040,0 };

        com_plan_CartsianPath(point_up_bottle_plan,3);

        robotiq_hand_move(32,5,50);
        ros::Duration(4).sleep();

        double back_point_plan[] = {0,0,0.15,0, point_up_collectBox[0],point_up_collectBox[1],point_up_collectBox[2],1};
        com_plan_CartsianPath(back_point_plan,2);

        robotiq_hand_move(85,5,50);

        ros::Duration(3).sleep();
    }
    return true;
}

bool UR_gripper::move_to_start_position(std::vector<double> joint_start_position) {
    move_group->setJointValueTarget(joint_start_position);
    move_group->move();
    return true;
}

Eigen::Vector3d UR_gripper::choose_fruit() {
    force_update_all_state();

    std::vector<KDL::Vector> body_markers = first_body_markers_;
    do{
        sub_motion_capture_ = false;
        ros::spinOnce();
        body_markers = first_body_markers_;
    }while(! sub_motion_capture_ && (body_markers.size() != 2) );
    Eigen::Vector3d marker1(body_markers[0][0], body_markers[0][1], body_markers[0][2]);
    Eigen::Vector3d marker2(body_markers[1][0], body_markers[1][1], body_markers[1][2]);


//    爆保证有目标物体
    std::vector<KDL::Frame> grasp_frame_bef_vec_tem;
    std::vector<double> box_rgb_vec_tem;
    do{
        force_update_all_state();
        grasp_frame_bef_vec_tem = grasp_frame_bef_vec;
        box_rgb_vec_tem = box_rgb_vec;
    }while( !(grasp_frame_bef_vec_tem.size()) && !(box_rgb_vec_tem.size()) );

    Eigen::Vector3d choosed_fruit(0,0,0);
    double length_point_line = 10;
    for(unsigned int i=0;i<box_rgb_vec.size();i = i+3)
    {
        Eigen::Vector3d tem_rgb_fruit = Eigen::Vector3d(box_rgb_vec_tem[i], box_rgb_vec_tem[i+1], box_rgb_vec_tem[i+2]) - marker1;
        Eigen::Vector3d cross_line = tem_rgb_fruit.cross(marker2-marker1);
        double length_point_line_tem = sqrt( cross_line.dot(cross_line) ) /sqrt( (marker2 - marker1).dot(marker2 - marker1) );
        if(length_point_line_tem < length_point_line)
        {
            length_point_line = length_point_line_tem;
            choosed_fruit = tem_rgb_fruit;
        }
    }
    return  choosed_fruit;

}

Eigen::Vector3d UR_gripper::calculate_center_point (std::vector<KDL::Vector> markers ){
    ROS_INFO_STREAM("marker size = " << markers.size());
    Eigen::Vector3d center_point(0,0,0) ;
    for (unsigned int i = 0; i < markers.size(); ++i) {
        // 如果丢点
        while (markers[i].data[0]>10){
            force_update_all_state();
            markers[i] = first_body_markers_[i];
        }
        for (int j = 0; j < 3; ++j) {
            center_point(j) += markers[i].data[j];

        }
    }
    double tmp = markers.size();
    center_point = 1.0/tmp* center_point;
//      std::cout<<center_point<<std::endl;
    return  center_point;
}

bool UR_gripper::huaman_interact() {
    force_update_all_state();
    KDL::Vector center_marker_point_tem(0,0,0);
    ros::Time start_marker = ros::Time::now();
    std::vector<KDL::Vector> body_markers = first_body_markers_;
    do{
        sub_motion_capture_ = false;
        ros::spinOnce();
        if(ros::Time::now()-start_marker > ros::Duration(2))
        {
            ROS_INFO_STREAM("can't get enough markers,the number of markers "<< body_markers.size());
            return false;
        }
    }while(! sub_motion_capture_ && (body_markers.size() < 2) );

    body_markers = first_body_markers_;

    for(unsigned int i=0; i<body_markers.size(); i++)
    {
//        如果丢点，用上一次的数据
        if(body_markers[i][0] > 10)
            break;
        center_marker_point_tem += body_markers[i];
    }
    center_marker_point_tem = center_marker_point_tem/body_markers.size();



    return true;
}

void UR_gripper::servoj_moveto(KDL::JntArray target, double time,bool wait_for_D) {
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

void UR_gripper::add_box(moveit_msgs::CollisionObject &collision_object, double bottle_height[],
                         double bottle_x_y_z[], KDL::Frame grasp_frame, std::string object_name,
                         std::string reference_frame = "base_link") {
    //        moveit_msgs::CollisionObject collision_object;move_group->getPlanningFrame();
    collision_object.header.frame_id = reference_frame;
    collision_object.id = object_name;


    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = bottle_height[0];
    primitive.dimensions[1] = bottle_height[1];
//    primitive.dimensions[2] = bottle_height;

    geometry_msgs::Pose bottle_pose;
//    tf::Quaternion bottle_qua;
//    double angle = 60/180.0 * M_PI;
//    bottle_qua.setRPY(angle,0,0);
//    bottle_pose.orientation.x = bottle_qua.x();
//    bottle_pose.orientation.y = bottle_qua.y();
//    bottle_pose.orientation.z = bottle_qua.z();
//    bottle_pose.orientation.w = bottle_qua.w();
//    bottle_pose.position.x = bottle_x_y_z[0];
//    bottle_pose.position.y = bottle_x_y_z[1];
//    bottle_pose.position.z = bottle_x_y_z[2];

    grasp_frame.M = grasp_frame.M * KDL::Rotation::RotX(M_PI_2);
    grasp_frame.M.GetQuaternion(bottle_pose.orientation.x, bottle_pose.orientation.y,
                                bottle_pose.orientation.z, bottle_pose.orientation.w);
    bottle_pose.position.x = grasp_frame.p.x();
    bottle_pose.position.y = grasp_frame.p.y();
    bottle_pose.position.z = grasp_frame.p.z();
//    bottle_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(bottle_pose);
    collision_object.operation = collision_object.ADD;

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.applyCollisionObject(collision_object);
    //    planning_scene_interface.addCollisionObjects(collision_objects);   //需要睡眠一下 否则rviz可能没有物体

//        return collision_object;
}

#endif //ros_myo_HUMAN_INTERACT_DEMO_H                                    
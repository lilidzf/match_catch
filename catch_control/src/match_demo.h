//
// Created by yang on 4/15/19. 研电赛
//

#ifndef CATCH_BOTTLE_MATCH_DEMO_H
#define CATCH_BOTTLE_MATCH_DEMO_H

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

class UR_gripper{

public:
    UR_gripper(ros::NodeHandle _nh,std::string PLANNING_GROUP){

        kinect2base_link.p = KDL::Vector(-1.06128558384, 0.067565534107, 0.717098767451);
        kinect2base_link.M = KDL::Rotation::Quaternion(-0.649109588692, 0.638784020982, -0.297812759475, 0.286215437566);


        base2ur_world.p = KDL::Vector(-0.01 ,0.377 ,0.808);
        base2ur_world.M = KDL::Rotation::RPY(0,0,1.570796327);

        move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        sub_hand_state = _nh.subscribe("/gripper/stat",1 , &UR_gripper::subHandStatesCB,this);
        sub_camera_rec = _nh.subscribe("/recognized_object_array",1 , &UR_gripper::subCamera,this);
        sub_camera_rgb = _nh.subscribe("/visualization_marker",1,&UR_gripper::subCameraRgb,this);
        sub_myo_hand_ = _nh.subscribe("/myo_raw/myo_emg",1,&UR_gripper::sub_myo_handCall,this);
        pub_hand_cmd_ = _nh.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd",1);

        sub_markers_ = _nh.subscribe("/vis_markers", 1, &UR_gripper::subMotionCaptureCB,this);
    }
    ~UR_gripper(){ delete move_group; }

    void subCamera(const object_recognition_msgs::RecognizedObjectArray& box_obj);

    void subCameraRgb(visualization_msgs::Marker marker);

    void subMotionCaptureCB(visualization_msgs::MarkerArray state);

    void sub_myo_handCall(ros_myo::EmgArray hand_state);

    void plan_CartesianPath();

    void com_plan_CartsianPath(double point[], int number_point);

    void robotiq_hand_move(float position,float vel, float force);

    void subHandStatesCB(robotiq_85_msgs::GripperStat state){
        hand_state_ = state;
        finish_hand_sub_ = true;
    }

    void force_update_all_state(){
        finish_hand_sub_ = false;
        do{
            ros::spinOnce();
        }while(!finish_hand_sub_);
    }

//    轨迹重定义
    void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale);

    void add_box(moveit_msgs::CollisionObject& collision_object,double bottle_height[],
            double bottle_x_y_z[], KDL::Frame grasp_frame, std::string object_name ,std::string reference_frame);

public:
//    KDL::Vector box_obj_position;
//    Eigen::Vector3d box_axis_z, grasp_point;
    KDL::Frame grasp_frame_bef;
    std::vector<KDL::Frame> grasp_frame_bef_vec;

    KDL::Frame kinect2base_link;
    KDL::Frame base2ur_world;

    moveit::planning_interface::MoveGroupInterface* move_group;
    bool get_box_frame = false;
    bool get_camera_rgb = false;
    double camera_box_position_rgb[3] = { 0.0, 0.0, 0.0};
    std::vector<double> box_rgb_vec;

    bool capture_point_ready = false;
    std::vector<double> capture_point_vec;

//    myo 控制手抓开合状态
    bool myo_grasp = false;

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
//    capture_point_ready = false;
    KDL::Frame base_link2map;
    base_link2map.p = KDL::Vector(0.310028, -0.412067 ,0.0397502);
    base_link2map.M = KDL::Rotation::Quaternion(0.000413926 ,-0.000457995 ,-0.704761 ,0.709444);

    capture_point_vec.clear();
    std::vector<visualization_msgs::Marker> markers = state.markers;
//    if(markers.size() == 1)
//    {
    KDL::Vector target_point(markers.at(4).pose.position.x, markers.at(4).pose.position.y, markers.at(4).pose.position.z);
    KDL::Vector point2base = base_link2map.Inverse() * target_point;
    capture_point_vec.push_back(point2base.x());
    capture_point_vec.push_back(point2base.y());
    capture_point_vec.push_back(point2base.z());
    capture_point_ready = true;
//    ROS_INFO_STREAM("point2base "<<point2base.x()<<" "<<point2base.y()<<" "<<point2base.z() );
//    }
//    else capture_point_ready = false;
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
//    ROS_INFO_STREAM("grasp or not "<<myo_grasp);
//    loop_rate.sleep();
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

void UR_gripper::plan_CartesianPath() {
    geometry_msgs::Pose target_pose3 = move_group->getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose3);

    do{
        ros::spinOnce();
    }while(! get_box_frame);

    KDL::Frame grasp_frame_use = grasp_frame_bef;

    grasp_frame_use.M.GetQuaternion(target_pose3.orientation.x, target_pose3.orientation.y,
                                    target_pose3.orientation.z, target_pose3.orientation.w);
//    target_pose1.position.x = grasp_frame.p.x() - hand_bias_z * grasp_frame.M.UnitZ().x();
    target_pose3.position.x = grasp_frame_use.p.x() - 0.13 * grasp_frame_use.M.UnitZ().x();
    target_pose3.position.y = grasp_frame_use.p.y() - 0.13 * grasp_frame_use.M.UnitZ().y();
    target_pose3.position.z = grasp_frame_use.p.z() - 0.13 * grasp_frame_use.M.UnitZ().z();

//    target_pose3.position.z += 0.06;
//    waypoints.push_back(target_pose3);  // down
//
//    target_pose3.position.x = 0.4;
//    target_pose3.position.y = 0.1;
//    target_pose3.position.z = 1.2;
//    waypoints.push_back(target_pose3);

//    target_pose3.position.z -= 0.05;
//    waypoints.push_back(target_pose3);

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
        ros::Duration(3).sleep();
    }

    ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

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

void UR_gripper::scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale) {
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

#endif //CATCH_BOTTLE_MATCH_DEMO_H

//
// Created by yang on 4/11/19.  需要同时启动ur5 和robotiq
//
#ifndef CATCH_BOTTLE_JOINT_STATE_MAINTAIN_H
#define CATCH_BOTTLE_JOINT_STATE_MAINTAIN_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class Joint_state{

public:
    Joint_state(ros::NodeHandle _nh){nh_ = _nh;

    robotiq_joint_state_sub =
            nh_.subscribe("robotiq/joint_states",1,&Joint_state::subJointStatesCB_robotiq,this);

    UR5e_joint_state_sub =
            nh_.subscribe("UR5e/joint_states",1,&Joint_state::subJointStatesCB_UR5e,this);

    UR5e_robotiq_joint_state_pub =
            nh_.advertise<sensor_msgs::JointState>("moveit_group/joint_states",1,this);

    ROS_INFO_STREAM("initialize the joint state ");
    }


    void subJointStatesCB_robotiq(const sensor_msgs::JointState& msgs) { gripper_joint_msgs = msgs; sub_gripper_finish=true;}

    void subJointStatesCB_UR5e(const sensor_msgs::JointState& msgs);


private:

    sensor_msgs::JointState gripper_joint_msgs,ur5e_joined_joint_msgs;
    ros::NodeHandle nh_;

    bool sub_gripper_finish = false;
    ros::Subscriber robotiq_joint_state_sub;
    ros::Subscriber UR5e_joint_state_sub;
    ros::Publisher UR5e_robotiq_joint_state_pub;
};

void Joint_state::subJointStatesCB_UR5e(const sensor_msgs::JointState &msgs) {
    if(sub_gripper_finish)
    {
        ur5e_joined_joint_msgs = msgs;
//        ROS_INFO_STREAM("get ur5r joint value "<<ur5e_joined_joint_msgs.position[0]);
        for(unsigned int i=0;i<msgs.name.size();i++)
        {
            ur5e_joined_joint_msgs.name.push_back(gripper_joint_msgs.name[i]);
            ur5e_joined_joint_msgs.position.push_back(gripper_joint_msgs.position[i]);
//            ur5e_joined_joint_msgs.velocity.push_back(gripper_joint_msgs.velocity[i]);   //没接实际手抓时没有速度,报错
        }
        UR5e_robotiq_joint_state_pub.publish(ur5e_joined_joint_msgs);
    }
}

#endif //CATCH_BOTTLE_JOINT_STATE_MAINTAIN_H

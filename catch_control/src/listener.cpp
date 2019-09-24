#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "robotiq_85_msgs/GripperCmd.h"
#include "robotiq_85_msgs/GripperStat.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "catch_bottle.h"
#include <math.h>
#include <stdlib.h>
#include <geometry_msgs/WrenchStamped.h>


//  用于采集六维力传感器数据和薄膜压力传感器数据  用于标定薄膜压力传感器数据

double real_pos,force_t,force_z;
float control_pos;
ros::Publisher pub_hand_cmd_;

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

    if(true ){
        hand_cmd.position = position;   // scale 0-0.085mm
        hand_cmd.speed = vel;
        hand_cmd.force = force;
        pub_hand_cmd_.publish(hand_cmd);
    }
}

void gripperstatCallback(const robotiq_85_msgs::GripperStat msg)
{
    real_pos = msg.position;
}

void etherdaqCallback(const geometry_msgs::WrenchStamped msg)
{
    force_z= msg.wrench.force.z;
}

void readCallback(const std_msgs::String::ConstPtr& msg)
{
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
   force_t = strtod(msg->data.c_str(), nullptr);
   ROS_INFO("I see: [%f]", force_t);
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    pub_hand_cmd_ = n.advertise<robotiq_85_msgs::GripperCmd>("gripper/cmd", 1);
    ros::Subscriber sub = n.subscribe("read", 1, readCallback);
    ros::Subscriber sub_gripper = n.subscribe("gripper/stat", 1, gripperstatCallback);
    ros::Subscriber sub_gripper_data = n.subscribe("ethdaq_data", 1, etherdaqCallback);

    ros::Rate loop_rate(30);
    ros::spinOnce();
    robotiq_hand_move(80,20,10);
    ros::spinOnce();

    ros::AsyncSpinner spinner(3);
    spinner.start();


    int name_suffix = 0;

    ROS_INFO("record data");
    control_pos=61.5;
    while(control_pos>56) {
        control_pos=control_pos-0.1;
        robotiq_hand_move(control_pos,20,10);
        name_suffix++;
        uint64_t nsec0 = ros::Time::now().toNSec();
        while (ros::ok()) {

            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
            double time_now_d = time_now / 1e9;
            //record
            std::ofstream file3;
            std::ostringstream oss;
            oss << name_suffix;
            std::string filename = oss.str() + "_force_test";
            std::string file = "/home/yang/catch_bottle/catch_control/record_servoj_sim/" + filename + ".txt";
            file3.open(file.c_str(), std::ios::app);
            file3 << time_now_d << " ";
            file3 << real_pos << " ";
            file3 << control_pos << " ";
            file3 << force_z << " ";
            file3 << force_t << " ";
            file3 << std::endl;
            file3.close();

            loop_rate.sleep();
            if (time_now_d > 2)
                break;
        }
        loop_rate.sleep();
    }
    spinner.stop();
    ros::waitForShutdown();

    return 0;
}
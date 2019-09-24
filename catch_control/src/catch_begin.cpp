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

//  根据实际采集得到的数据计算实际力

double real_pos,real_force,force_t,deta_pos,force_1;
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
void readCallback(const std_msgs::String::ConstPtr& msg)
{
    force_t = strtod(msg->data.c_str(), nullptr);
//    real_force = -2.939*pow(10,5)*exp(-0.004684*force_t)+2.939*pow(10,5)*exp(-0.004679*force_t);
    real_force = 0.2595 + 0.9564 * force_t;
    ROS_INFO("I heard: [%f]",real_force);
}
void write_to_file()
{
        ros::Time time = ros::Time::now();
        int name_suffix = 5;
        std::ofstream file3;
        std::ostringstream oss;
        oss << name_suffix;
        std::string filename = oss.str() + "_fcatch_begin";
        std::string file = "/home/jp/catkin_ws_1/src/catch_control/record_force_test/1/" + filename + ".txt";
        file3.open(file.c_str(), std::ios::app);
        file3 << time << " ";
        file3 << real_pos << " ";
        file3 << control_pos << " ";
        file3 << force_1<< " ";
        file3 << std::endl;
        file3.close();
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "catch_begin");
    ros::NodeHandle n;

    pub_hand_cmd_ = n.advertise<robotiq_85_msgs::GripperCmd>("gripper/cmd", 1);
    ros::Subscriber sub_gripper = n.subscribe("gripper/stat", 1, gripperstatCallback);
    ros::Subscriber sub_force = n.subscribe("read", 1, readCallback);

    ros::Rate loop_rate(30);
    ROS_INFO("catch begin");
    control_pos = 85;
    float real_force_0 = 0;
    while (ros::ok()) {
        control_pos = control_pos - 0.3;
        robotiq_hand_move(control_pos, 20, 10);
        deta_pos = real_pos * 1000 - control_pos;
        loop_rate.sleep();
        force_1 = real_force - real_force_0;
        write_to_file();
        if (force_1 > 2) {
            ROS_INFO("break");
            break;
        }
        if (deta_pos > 6 || control_pos < 0) {
            ROS_INFO("position is wrong");
            break;
        }
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}

//
// Created by yang on 5/14/19.
//
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

void callback(sensor_msgs::JointStateConstPtr jointptr);

int main(int argc, char** argv) {
    ros::init(argc, argv, "catch_control");
    ros::NodeHandle nh_;
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    ros::Subscriber sub = nh_.subscribe("/UR5e/joint_states",1,&callback);
    ros::waitForShutdown();
}

void callback(sensor_msgs::JointStateConstPtr jointptr){
    ros::Rate loop_rate(10);
//    ros::Time start_time = jointptr->header.stamp;


    //record
    std::ofstream ur_joinst_file;
    std::ostringstream oss;
    int name_suffix = 1;
    oss << name_suffix;
    std::string folder = "/home/yang/catch_bottle/catch_control/record_servoj_sim/";
    ur_joinst_file.open((folder + oss.str() + "position" + ".txt").c_str(), std::ios::app);

//    uint64_t nsec0 = ros::Time::now().toNSec();
//    uint64_t time_now = ros::Time::now().toNSec() - nsec0;
//    double time_now_d = time_now / 1e9;
    for(unsigned int i=0;i<jointptr->position.size();i++)
    {
        ur_joinst_file << jointptr->header.stamp.toSec()/pow(10,9)<<" "<<jointptr->position[i]<<" "
                        << jointptr->velocity[i]<<std::endl;
        ur_joinst_file.close();
    }
    loop_rate.sleep();
}

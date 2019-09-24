//
// Created by yang on 3/31/19.
//

#include "joint_state_maintain.h"


int main(int argc,char **argv)
{
    ros::init(argc,argv,"joint_state_maintain");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    Joint_state UR5e_iq_jointstate(nh);

    ros::waitForShutdown();
}


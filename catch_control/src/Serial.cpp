
//
// Created by dzf on 19-9-19.
//
#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>
//#include <std_msgs/Float32.h>
#include <iostream>

serial::Serial ser; //声明串口对象

//回调函数
void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" <<msg->data);
    ser.write(msg->data);   //发送串口数据
}

void print_value_screen(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("SensorValue: %s", msg->data.c_str());
    ROS_INFO("get_sensor_value\n");
}
int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;

    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("air_sensor_value", 1000);

    ros::Subscriber sensor_value_sub = nh.subscribe("air_sensor_value", 1000, print_value_screen);
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    //指定循环的频率
    ros::Rate loop_rate(50);
    while(ros::ok())
    {

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port\n");
            std_msgs::String result;
            result.data = ser.read(ser.available());
//            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("SensorValueGet: " << result.data);
            read_pub.publish(result);
        }
        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();

    }
}


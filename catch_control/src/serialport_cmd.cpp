#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>

//  串口采集数据

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
    ROS_INFO("GetSensorValue\n");
}
//
int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle np;
    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = np.subscribe("hand_cmd", 1, write_callback);
    //发布主题
    ros::Publisher read_pub = np.advertise<std_msgs::String>("air_sensor_value", 1);

    ros::Subscriber SensorValueSub = np.subscribe("air_sensor_value", 1 ,print_value_screen);
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
            ROS_INFO_STREAM("GetSensorValue: " << result.data);
            read_pub.publish(result);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
//    ros::Rate loop_rate(30);
//    while(ros::ok())
//    {
//        ROS_INFO_STREAM("Waiting for read or write");
//        std::string writing = "3200 0";
//        ROS_INFO_STREAM("Writing...");
//        ser.write(writing);
//        std::cout << "Writing: " << writing << "\n";
//        ros::Duration(5).sleep();
//
////        }
//        //处理ROS的信息，比如订阅消息,并调用回调函数
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//    while(ros::ok())
//    {
//        ros::spin();
//    }
//    ros::waitForShutdown();
}

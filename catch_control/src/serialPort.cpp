#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

//  串口采集数据

serial::Serial ser; //声明串口对象
//回调函数
void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" <<msg->data);
    ser.write(msg->data);   //发送串口数据
}
int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("write", 1, write_callback);
    //发布主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1);
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(57600);
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
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port\n");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);

//            //获取缓冲区内的字节数
//            size_t n = ser.available();
//            if(n!=0)
//            {
//                uint8_t  buffer[1024];
//                //读出数据
//                n = ser.read(buffer, n);
//
//                for(int i=0; i<n; i++)
//                {
//                    //16进制的方式打印到屏幕
//                    std::cout << std::hex << (buffer[i] & 0xff) << " ";
//                }
//                std::cout << std::endl;
//                //把数据发送回去
//                ser.write(buffer, n);

        }

        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();
    }
}
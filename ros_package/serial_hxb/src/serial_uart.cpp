/*
  hujianjun
*/
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
//#include <string>
serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
   ROS_INFO_STREAM("Writing to serial port " << msg->data);
   ser.write(msg->data);
 //  ROS_INFO_STREAM("Writing to serial port");

}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_uart");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 4000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

        ros::Rate loop_rate(5);

    while(ros::ok())
    {

        ros::spinOnce();
        int read_buffer_num;

        std_msgs::String read_buffer;
        std_msgs::String end_data ;
        end_data.data = "\n";

        read_buffer_num = ser.readline(read_buffer.data,2000,end_data.data);

        char ch = read_buffer.data[read_buffer.data.length()-1];
        if(ch=='\n')
        {
        read_pub.publish(read_buffer);

        ROS_INFO_STREAM("read_buffer_num "<< read_buffer_num);
        ROS_INFO_STREAM("read_buffer "<< read_buffer.data);
        }
        // if(ser.available()){
        //     ROS_INFO_STREAM("Reading from serial port");
        //     std_msgs::String result;
        //     result.data = ser.read(ser.available());
        //     ROS_INFO_STREAM("Read: " << result.data);
        //     read_pub.publish(result);
        // }
        //   loop_rate.sleep();

   }
}

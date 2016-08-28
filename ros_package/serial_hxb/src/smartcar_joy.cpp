#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


//ros::Rate loop_rate(3);

class Teleopsmartcar
{
public:
  Teleopsmartcar();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void callback(const ros::TimerEvent&);
  ros::NodeHandle nh_;
  ros::Timer timer;

  int linear_x,linear_y, angular_z;
  double l_scale_x,l_scale_y, a_scale_z;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  geometry_msgs::Twist vel;
};


Teleopsmartcar::Teleopsmartcar():
  linear_x(4),
  linear_y(3),
  angular_z(0)
{

  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("axis_angular_z", angular_z, angular_z);
  nh_.param("scale_angular_z", a_scale_z, a_scale_z);
  nh_.param("scale_linear_x", l_scale_x, l_scale_x);
  nh_.param("scale_linear_y", l_scale_y, l_scale_y);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  timer = nh_.createTimer(ros::Duration(0.33),&Teleopsmartcar::callback,this);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &Teleopsmartcar::joyCallback, this);

}

void Teleopsmartcar::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // geometry_msgs::Twist vel;
  vel.angular.z = a_scale_z*joy->axes[angular_z];
  vel.linear.x = l_scale_x*joy->axes[linear_x];
  vel.linear.y = l_scale_y*joy->axes[linear_y];
  // vel_pub_.publish(vel);
//  loop_rate.sleep();
}

void Teleopsmartcar::callback(const ros::TimerEvent&)
{
 vel_pub_.publish(vel);
 }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smartcar_joy");
  Teleopsmartcar Teleopsmartcar;
  ros::spin();
}

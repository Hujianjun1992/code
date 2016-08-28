#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <serial_hxb/SetDriveControlGains.h>
#include <string>

using namespace std;

class Freescale
{
public:
  char message[80];
  vector<string> sep;
  long Count;
  stringstream s_s;
  std_msgs::String msg;
  float velocityPParam;
  float velocityIParam;
  float turnPParam;
  float turnIParam;
  float commandTimeout;
  vector<float> speedControllerParams;//(5,0.0);
  vector<int> speedControllerParamstoInt;//(10.0);
  vector<float> twistCommandParams;//(2,0.0);
  vector<int> twistCommandParamstoInt;//(4,0);
  Freescale();
  // virtual ~Freescale();
  vector<string> split(string str,char delimiter);
  vector<int> GetBaseAndExponent(float floatValue,int resolution = 4);
  vector<int> GetBaseAndExponents(vector<float> floatValues);

private:
  ros::NodeHandle nh;
  ros::Subscriber Serial_sub;
  ros::Publisher Serial_pub;
  ros::Subscriber Cmd_vel_sub;
  ros::Publisher Odometry_pub;
  ros::ServiceServer SetDriveControlGainsService;
  tf::TransformBroadcaster OdometryTransformBroadcaster;
  void read_callback(const std_msgs::String::ConstPtr& msg);
  void Cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twistCommand);
  void InitializeDriveGeometry();
  void InitializeSpeedController();
  void InitializeBatteryMonitor();
  void BroadcastOdometryInfo(const vector<string>& sep);
  float GetCommandTimeoutForSpeedController();
  bool SetDriveControlGains_callback(serial_hxb::SetDriveControlGainsRequest& request,
					    serial_hxb::SetDriveControlGainsResponse& response);
};

void Freescale::Cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twistCommand)
{
  float v = 0.0;
  float omega = 0.0;

  v = twistCommand->linear.x ;
  omega = twistCommand->angular.z ;

  twistCommandParams[0] = v;
  twistCommandParams[1] = omega;

  twistCommandParamstoInt = GetBaseAndExponents(twistCommandParams);

  s_s << "s ";

  for(unsigned int i = 0; i < twistCommandParamstoInt.size(); ++i)
    {
      s_s << twistCommandParamstoInt[i] << " ";
    }

  s_s << "\n" ;

  msg.data = s_s.str();
  s_s.str("");

  ROS_INFO("SpeedControllerParams : %s",msg.data.c_str());

  Serial_pub.publish(msg);
}

void Freescale::read_callback(const std_msgs::String::ConstPtr& msg)
{
   sep = split(msg->data,' ');

  if(sep[0]=="InitializeDriveGeometry")
    {
      InitializeDriveGeometry();
      return ;
    }
  if(sep[0]=="InitializeSpeedController")
    {

      InitializeSpeedController();
      return ;
    }
  if(sep[0]=="InitializeBatteryMonitor")
    {
      InitializeBatteryMonitor();
      return ;
    }
  if(sep[0]=="o")
    {
       BroadcastOdometryInfo(sep);
      return ;
    }
}

bool Freescale::SetDriveControlGains_callback(serial_hxb::SetDriveControlGainsRequest& request,
				   serial_hxb::SetDriveControlGainsResponse& response)
{
  response.data = "ROS by Hxb !" ;

  velocityPParam = request.velocityPParam;
  velocityIParam = request.velocityIParam;
  turnPParam = request.turnPParam;
  turnIParam = request.turnIParam;

  commandTimeout = GetCommandTimeoutForSpeedController();

  speedControllerParams[0] = velocityPParam ;
  speedControllerParams[1] = velocityIParam ;
  speedControllerParams[2] = turnPParam ;
  speedControllerParams[3] = turnIParam ;
  speedControllerParams[4] = commandTimeout ;

  speedControllerParamstoInt = GetBaseAndExponents(speedControllerParams);

  s_s << "SpeedControllerParams ";

  for(unsigned int i = 0; i < speedControllerParamstoInt.size(); ++i)
    {
      s_s << speedControllerParamstoInt[i] << " ";
    }

  s_s << "\n" ;

  msg.data = s_s.str();
  s_s.str("");

  ROS_INFO("SpeedControllerParams : %s",msg.data.c_str());

  Serial_pub.publish(msg);

  return true;
 }

void Freescale::InitializeDriveGeometry()
{
  float wheelDiameter;
  float trackWidth;
  int countsPerRevolution;
  vector<int> wheelDiameterParts(2,0);
  vector<int> trackWidthParts(2,0);

  nh.getParam("driveGeometry/trackWidth",trackWidth);
  nh.getParam("driveGeometry/wheelDiameter",wheelDiameter);
  nh.getParam("drive/countsPerRevolution",countsPerRevolution);

  wheelDiameterParts = GetBaseAndExponent(wheelDiameter);
  trackWidthParts = GetBaseAndExponent(trackWidth);

 //s_s << "DriveGeometry ";

  sprintf(message,"DriveGeometry %d %d %d %d\n",wheelDiameterParts.at(0),wheelDiameterParts.at(1),trackWidthParts.at(0),trackWidthParts.at(1));

  msg.data=message;

  Serial_pub.publish(msg);
}

void Freescale::InitializeSpeedController()
{
  nh.getParam("speedController/velocityPParam",velocityPParam);
  nh.getParam("speedController/velocityIParam",velocityIParam);
  nh.getParam("speedController/turnPParam",turnPParam);
  nh.getParam("speedController/turnIParam",turnIParam);
  nh.getParam("speedController/commandTimeout",commandTimeout);

  speedControllerParams[0] = velocityPParam ;
  speedControllerParams[1] = velocityIParam ;
  speedControllerParams[2] = turnPParam ;
  speedControllerParams[3] = turnIParam ;
  speedControllerParams[4] = commandTimeout ;

  speedControllerParamstoInt = GetBaseAndExponents(speedControllerParams);

   s_s << "SpeedControllerParams ";

   for(unsigned int i = 0; i < speedControllerParamstoInt.size(); ++i)
     {
       s_s << speedControllerParamstoInt[i] << " ";
     }

   s_s << "\n" ;

   msg.data = s_s.str();
   s_s.str("");

   ROS_INFO("SpeedControllerParams : %s",msg.data.c_str());

   Serial_pub.publish(msg);

}

void Freescale::InitializeBatteryMonitor()
{

}

void Freescale::BroadcastOdometryInfo(const vector<string>& sep)
{
         int partsCount = sep.size();

  if(partsCount < 6)
    return ;

  // float x = 0.0;
  float x = stof(sep[1]);
  float y = stof(sep[2]);
  float theta = stof(sep[3]);
  float vx = stof(sep[4]);
  float vy = 0.0 ;
  float omega = stof(sep[5]);
  ros::Time rosNow ;

  geometry_msgs::Quaternion quaternion;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);
  geometry_msgs::TransformStamped Odometry_trans;
  Odometry_trans.header.stamp = rosNow ;
  Odometry_trans.transform.translation.x = x ;
  Odometry_trans.transform.translation.y = y ;
  Odometry_trans.transform.translation.z = 0.0 ;
  Odometry_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta) ;
  
  nav_msgs::Odometry odometry;
  tf::Transform transform;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(theta/2.0);
  quaternion.w = cos(theta/2.0);
  rosNow = ros::Time::now();

  transform.setOrigin(tf::Vector3(x,y,0));
  transform.setRotation(tf::Quaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w));
  OdometryTransformBroadcaster.sendTransform(tf::StampedTransform(transform,rosNow,"odom","base_link"));
  // OdometryTransformBroadcaster.sendTransform(tf::StampedTransform(
                                             // tf::Transform(tf::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                                             // tf::Vector3(x, y, 0)),rosNow,"base_link","odom"));
         // OdometryTransformBroadcaster.sendTransform((x,y,0),(quaternion.x,quaternion.y,quaternion.z,quaternion.w),rosNow,"base_link","odom");
  // Odometry_trans.transform.translation.x = x;
  // Odometry_trans.transform.translation.y = y;
  // Odometry_trans.transform.translation.z = 0.0;
  // Odometry_trans.transform.rotation = quaternion ;

  // OdometryTransformBroadcaster.sendTransform(Odometry_trans);
         // OdometryTransformBroadcaster.sendTransform
  odometry.header.stamp = rosNow ;
  odometry.header.frame_id = "odom" ;
  //set the position
  odometry.pose.pose.position.x = x ;
  odometry.pose.pose.position.y = y ;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation = quaternion ;
  //set the velocity
  odometry.child_frame_id = "base_link";
  odometry.twist.twist.linear.x = vx ;
  odometry.twist.twist.linear.y = vy ;
  odometry.twist.twist.angular.z = omega ;
  //publish the message
  Odometry_pub.publish(odometry);
         //  ROS_INFO_STREAM("hujianjun2");
}

Freescale::Freescale():
speedControllerParams(5,.0),
speedControllerParamstoInt(10.0),
twistCommandParams(2,0.0),
twistCommandParamstoInt(4,0)
{
  velocityPParam = 0.0;
  velocityIParam = 0.0;
  turnPParam = 0.0;
  turnIParam = 0.0;
  commandTimeout=0.5 ;

  Serial_sub = nh.subscribe("read",1000,&Freescale::read_callback,this);
  Serial_pub = nh.advertise<std_msgs::String>("write",1000);

  Cmd_vel_sub = nh.subscribe("cmd_vel",1000,&Freescale::Cmd_vel_callback,this);

  Odometry_pub = nh.advertise<nav_msgs::Odometry>("odom",50);

  SetDriveControlGainsService = nh.advertiseService("setDriveControlGains",&Freescale::SetDriveControlGains_callback,this);

}

vector<string> Freescale::split(string str,char delimiter)
{
  vector<string> internal;
  stringstream ss(str);
  string tok ;

 while(getline(ss,tok,delimiter))
   {
     internal.push_back(tok);
   }
 return internal;
}

vector<int> Freescale::GetBaseAndExponent(float floatValue,int resolution)
{
 int exponent = 0 ;
 int multipler = 0 ;
 int base = 0;
 vector<int> BaseAndExponnent(2,0);

 if(floatValue == 0.0)
   return BaseAndExponnent;
 else
   {
     exponent = int(1.0+log10(abs(floatValue)));
     multipler = int(pow(10,resolution-exponent));
     base = int(floatValue*multipler);
     BaseAndExponnent[0]=base;
     BaseAndExponnent[1]=exponent-resolution;
     return BaseAndExponnent;
   }
}

vector<int> Freescale::GetBaseAndExponents(vector<float> floatValues)
{
 vector<int> baseAndExponents;
 vector<int> baseAndExponent(2);
 for(unsigned int i = 0 ;i < floatValues.size(); ++i)
   {
     baseAndExponent = GetBaseAndExponent(floatValues[i]);
     baseAndExponents.push_back(baseAndExponent[0]);
     baseAndExponents.push_back(baseAndExponent[1]);
   }
 return baseAndExponents ;
}

float Freescale::GetCommandTimeoutForSpeedController()
{
 float commandTimeout = 0.5;

 nh.getParam("speedController/commandTimeout",commandTimeout);

 return commandTimeout ;
}


int main (int argc,char** argv)
{
 ros::init(argc,argv,"freescale");

 Freescale freescale;

 ros::spin();
}

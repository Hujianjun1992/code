#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <cstring>
#include <vector>
#include <cmath>

using namespace std;

//ros::NodeHandle nh;

// void GetBaseAndE// xponent(float Param_float)
// {

// }

vector<int> GetBaseAndExponent(float floatValue,int resolution=4)
{
  int exponent=0;
  int multipler=0;
  int base = 0;
  vector<int> BaseAndExponnent(2,0);
  if(floatValue == 0.0)
    return BaseAndExponnent;
  else
    {
      exponent = int(1.0+log10(abs(floatValue)));
      //      cout << exponent << endl ;
      multipler =int(pow(10,resolution-exponent));
      //      cout << multipler << endl ;
      base = int(floatValue*multipler);
      BaseAndExponnent[0]=base;
      BaseAndExponnent[1]=exponent-resolution;
      return BaseAndExponnent;
    }

}


void InitializeDriveGeometry()
{
  float wheelDiameter=52.35;
  float trackWidth=0.0;
  int  countsPerRevolution;
  vector<int> wheelDiameterParts(2,0);
  vector<int> trackWidthParts(2,0);
  // vector<int> countsPerRevolution
  char message[80];
  ros::NodeHandle nh;

  nh.getParam("driveGeometry/trackWidth",trackWidth);
  nh.getParam("driveGeometry/wheelDiameter",wheelDiameter);
  nh.getParam("driveGeometry/countsPerRevolution",countsPerRevolution);

  //   cout << wheelDiameter  << endl ;

  wheelDiameterParts = GetBaseAndExponent(wheelDiameter);
  trackWidthParts = GetBaseAndExponent(trackWidth);

  // cout << wheelDiameterParts[0]  << endl ;
  // cout << wheelDiameterParts[1]  << endl ;

  // countsPerRevolution = GetBaseAndExponent(countsPerRevolution);
  sprintf(message,"DriveGeometry %d %d %d %d\n",wheelDiameterParts.at(0),wheelDiameterParts.at(1),trackWidthParts.at(0),trackWidthParts.at(1));
//  message = "DriveGeometry %d %d %d %d %d\n",(wheelDiameterParts[0],wheelDiameterParts[1],trackWidthParts[0],trackWidthParts[1],countsPerRevolution);

   cout << message << endl ;
   //  ROS_INFO("Sending drive geometry params message: %s" , message.c_str());
}

vector<string> split(string str,char delimiter)
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

uint32_t Count=0;

class  Freescale
{
public:
  //long Count = 0 ;
  void read_callback(const std_msgs::String::ConstPtr& msg);
  Freescale();
};


// void split(char* string)
// {
//   int i=0;
//   char *Line[15];
//   char *tockenPtr = strtok(string," ");

//   while(tockenPtr!=NULL){
//     Line[i] = tockenPtr;
//     tockenPtr = strtok(NULL," ");
//     i++;
//   }
//   //  ROS_INFO_STREAM(Line);
//   // ROS_INFO("%s",*Line);
// }

void Freescale::read_callback(const std_msgs::String::ConstPtr& msg)
{
  // split((char*)msg->data.c_str());
  vector<string> sep = split(msg->data,' ');

  for(unsigned int i = 0;i < sep.size(); ++i)
    {
      // ROS_INFO("%s",sep[i]);
       // cout << sep[i] << endl ;
    }
  Count++;
  if(sep[0]=="InitializeDriveGeometry")
    {
      InitializeDriveGeometry();
    }
  // std::stringstream ss;

  //[ INFO] [1464546536.284996677]: hujianjun :10
  // ss << msg.data.c_str << Count++;
  // ROS_INFO("%d %s",Count,msg->data.c_str());
}

Freescale::Freescale()
{

}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "freescale");

  ros::NodeHandle nh ;

  Freescale freescale;

  ros::Subscriber Serial_sub = nh.subscribe("read",1000,&Freescale::read_callback,&freescale);

  while(ros::ok())
  {
  ros::spin();
  }
  return 0;
}

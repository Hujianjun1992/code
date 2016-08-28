# pragma once

#include <fstream>
#include <vector>
#include <map>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct CAMERA_INTRINSIC_PARAMETERS
{
  double cx, cy,fx,fy,scale;
};

PointCloud::Ptr image2PointCloud(cv::Mat& rgb,cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);

struct FRAME
{
  cv::Mat rgb, depth;
  cv::Mat desp;
  //  vector<cv::KeyPoint> kp1,kp2;

  vector<cv::KeyPoint> kp;
};

struct RESULT_OF_PNP
{
  cv::Mat rvec, tvec;
  int inliers;
};

void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor);

RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);


class ParameterReader
{
 public:
  ParameterReader(string filename = "./parameters.txt")
    {
      ifstream fin(filename.c_str());
      if (!fin ) {
        cerr << "parameter file does not exist." << endl;
        return;
      }
      while (!fin.eof()) {
        string str;
        getline(fin,str);
        if (str[0] == '#') {
          continue;
        }
        int pos = str.find("=");
        if (pos == -1) {
          continue;
        }
        string key = str.substr(0, pos);
        string value = str.substr(pos+1, str.length());
        data[key] = value ;
        cout << data[key] << endl;

        if (!fin.good()) {
          break;
        }
      }
    }
  string getData(string key)
  {
    map<string,string>::iterator iter = data.find(key);
    if (iter == data.end()) {
      cerr << "Parameter name" << key <<" not found!" << endl;
      return string("NOT_FOUND");
    }
    return iter->second;
  }
 public:
  map<string,string>data;
};

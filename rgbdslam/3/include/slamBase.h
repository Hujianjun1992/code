# pragma once

#include <fstream>
#include <vector>
#include <map>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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


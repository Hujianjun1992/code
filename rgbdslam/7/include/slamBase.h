# pragma once

#include <fstream>
#include <vector>
#include <map>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>


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
  int frameID;
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

Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec);

PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera);

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
        //  cout << data[key] << endl;

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

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
  ParameterReader pd;
  CAMERA_INTRINSIC_PARAMETERS camera;
  camera.fx = atof ( pd.getData( "camera.fx" ).c_str() );
  camera.fy = atof ( pd.getData( "camera.fy" ).c_str() );
  camera.cx = atof ( pd.getData( "camera.cx" ).c_str() );
  camera.cy = atof ( pd.getData( "camera.cy" ).c_str() );
  camera.scale = atof ( pd.getData( "camera.scale" ).c_str() );

  return camera ;
}


//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

/*********************************************************************************

 *********************************************************************************/

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <eigen3/Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop

#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include <boost/filesystem.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>  

#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>

// 利用OpenCV实时显示视觉惯性里程计的轨迹在二维平面上的投影
class PoseViewer
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  constexpr static const double imageSize = 500.0;
  PoseViewer()
  {
    cv::namedWindow("OKVIS Top View");
    _image.create(imageSize, imageSize, CV_8UC3);
    drawing_ = false;
    showing_ = false;
  }
  // this we can register as a callback
  // 作用：生成image图片
  void publishFullStateAsCallback(
      const okvis::Time & /*t*/, const okvis::kinematics::Transformation & T_WS,
      const Eigen::Matrix<double, 9, 1> & speedAndBiases,
      const Eigen::Matrix<double, 3, 1> & /*omega_S*/)
  {
    
    // just append the path
    Eigen::Vector3d r = T_WS.r();
    Eigen::Matrix3d C = T_WS.C();
    _path.push_back(cv::Point2d(r[0], r[1]));
    _heights.push_back(r[2]);
    // maintain scaling
    if (r[0] - _frameScale < _min_x)
      _min_x = r[0] - _frameScale;
    if (r[1] - _frameScale < _min_y)
      _min_y = r[1] - _frameScale;
    if (r[2] < _min_z)
      _min_z = r[2];
    if (r[0] + _frameScale > _max_x)
      _max_x = r[0] + _frameScale;
    if (r[1] + _frameScale > _max_y)
      _max_y = r[1] + _frameScale;
    if (r[2] > _max_z)
      _max_z = r[2];
    _scale = std::min(imageSize / (_max_x - _min_x), imageSize / (_max_y - _min_y));

    // draw it
    while (showing_) {
    }
    drawing_ = true;
    // erase
    _image.setTo(cv::Scalar(10, 10, 10));
    drawPath();
    // draw axes
    Eigen::Vector3d e_x = C.col(0);
    Eigen::Vector3d e_y = C.col(1);
    Eigen::Vector3d e_z = C.col(2);
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
        cv::Scalar(0, 0, 255), 1, CV_AA);
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
        cv::Scalar(0, 255, 0), 1, CV_AA);
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
        cv::Scalar(255, 0, 0), 1, CV_AA);

    // some text:
    std::stringstream postext;
    postext << "position = [" << r[0] << ", " << r[1] << ", " << r[2] << "]";
    cv::putText(_image, postext.str(), cv::Point(15,15),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1);
    std::stringstream veltext;
    veltext << "velocity = [" << speedAndBiases[0] << ", " << speedAndBiases[1] << ", " << speedAndBiases[2] << "]";
    cv::putText(_image, veltext.str(), cv::Point(15,35),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1);

    drawing_ = false; // notify
    
  }
  
  void display()
  {
    while (drawing_) {
    }
    showing_ = true;
    cv::imshow("OKVIS Top View", _image);
    showing_ = false;
    cv::waitKey(1);
  }
 
 private:
  cv::Point2d convertToImageCoordinates(const cv::Point2d & pointInMeters) const
  {
    cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x, _min_y)) * _scale;
    return cv::Point2d(pt.x, imageSize - pt.y); // reverse y for more intuitive top-down plot
  }
  
  void drawPath()
  {
    for (size_t i = 0; i + 1 < _path.size(); ) {
      cv::Point2d p0 = convertToImageCoordinates(_path[i]);
      cv::Point2d p1 = convertToImageCoordinates(_path[i + 1]);
      cv::Point2d diff = p1-p0;
      if(diff.dot(diff)<2.0){
        _path.erase(_path.begin() + i + 1);  // clean short segment
        _heights.erase(_heights.begin() + i + 1);
        continue;
      }
      double rel_height = (_heights[i] - _min_z + _heights[i + 1] - _min_z)
                      * 0.5 / (_max_z - _min_z);
      cv::line(
          _image,
          p0,
          p1,
          rel_height * cv::Scalar(255, 0, 0)
              + (1.0 - rel_height) * cv::Scalar(0, 0, 255),
          1, CV_AA);
      i++;
    }
  }
  
  cv::Mat _image;
  std::vector<cv::Point2d> _path;
  std::vector<double> _heights;
  double _scale = 1.0;
  double _min_x = -0.5;
  double _min_y = -0.5;
  double _min_z = -0.5;
  double _max_x = 0.5;
  double _max_y = 0.5;
  double _max_z = 0.5;
  const double _frameScale = 0.2;  // [m]
  std::atomic_bool drawing_;
  std::atomic_bool showing_;
};

// 作用：1、结合ROS，发布视觉惯性里程计数据的类
//      2、发布实时里程计的tf信息
//      3、发布历史的轨迹
//      4、发布实时检测到的landmark

class ROSVIOPUB
{
public:
  ROSVIOPUB()
  {
    pclpub = nh.advertise<sensor_msgs::PointCloud2>("pcl_current",1);
    pclmappub = nh.advertise<sensor_msgs::PointCloud2>("pcl_map",1);
    pathpub = nh.advertise<nav_msgs::Path>("path",1);
  }
  ~ROSVIOPUB(){}
  
  // 可以注册到okvis后端的回调函数，可以从okvis的后端取回时间戳t，以及实时的坐标变换T
  void displayCallback(const okvis::Time &t, const okvis::kinematics::Transformation &T)
  {
    std::cout<<T.coeffs()<<std::endl;
    
    tf::Transform transform;
    tf::Vector3 r(T.coeffs()[0],T.coeffs()[1],T.coeffs()[2]);
    transform.setOrigin(r);
    tf::Quaternion q(T.coeffs()[3],T.coeffs()[4],T.coeffs()[5],T.coeffs()[6]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "imu"));
    
    geometry_msgs::PoseStamped posestamp;
    posestamp.pose.position.x = T.coeffs()[0];
    posestamp.pose.position.y = T.coeffs()[1];
    posestamp.pose.position.z = T.coeffs()[2];
    posestamp.pose.orientation.x = T.coeffs()[3];
    posestamp.pose.orientation.y = T.coeffs()[4];
    posestamp.pose.orientation.z = T.coeffs()[5];
    posestamp.pose.orientation.w = T.coeffs()[6];
    
    path.poses.push_back(posestamp);
    path.header.frame_id = "map";
    pathpub.publish(path);
       
  }
  
  // 可以注册到okvis后端的回调函数，可以从okvis的后端取回时间戳t，以及实时的坐标变换T-WS、T-SCs，以及可以取回speedandBias等等信息
  void fullstatewithextrinsicCallback(const okvis::Time &t,
				      const okvis::kinematics::Transformation &T_WS,
				      const Eigen::Matrix<double, 9, 1> &,
				      const Eigen::Matrix<double, 3, 1> &,
				      const std::vector<okvis::kinematics::Transformation,
				      Eigen::aligned_allocator<okvis::kinematics::Transformation>> &T_SCs)
  {
    tf::Transform transform;
    okvis::kinematics::Transformation T;
    T = T_WS*T_SCs[0];
    tf::Vector3 r(T.coeffs()[0],T.coeffs()[1],T.coeffs()[2]);
    transform.setOrigin(r);
    tf::Quaternion q(T.coeffs()[3],T.coeffs()[4],T.coeffs()[5],T.coeffs()[6]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera"));
    
    geometry_msgs::PoseStamped posestamp;
    posestamp.pose.position.x = T.coeffs()[0];
    posestamp.pose.position.y = T.coeffs()[1];
    posestamp.pose.position.z = T.coeffs()[2];
    posestamp.pose.orientation.x = T.coeffs()[3];
    posestamp.pose.orientation.y = T.coeffs()[4];
    posestamp.pose.orientation.z = T.coeffs()[5];
    posestamp.pose.orientation.w = T.coeffs()[6];
    
    path.poses.push_back(posestamp);
    path.header.frame_id = "map";
    pathpub.publish(path);
  }
  
  // 可以注册到okvis后端的回调函数，可以从okvis的后端取回时间戳t，以及实时的landmark
  // landmarkCallback的回调函数，其中第一个参数是时间，第二个是当前的landmark，第三个是marg掉的landmark
  void landmarkCallback(const okvis::Time &t, const okvis::MapPointVector &landmarks,const okvis::MapPointVector &marglandmarks)
  {
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;
    for(okvis::MapPointVector::const_iterator it = landmarks.begin();it != landmarks.end() ;it++)
    {
      point.x = it->point[0];
      point.y = it->point[1];
      point.z = it->point[2];
      cloud->push_back(point);
      
    }
    cloud->width = landmarks.size();
    cloud->height= 1;
    //globalmap = globalmap+*cloud;
    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud,output);
    output.header.stamp.sec = t.sec;
    output.header.stamp.nsec = t.nsec;
    output.header.frame_id = "map";
    pclpub.publish(output);
    /*
    pcl::toROSMsg(globalmap,output);
    output.header.stamp.sec = t.sec;
    output.header.stamp.nsec = t.nsec;
    output.header.frame_id = "map";
    pclmappub.publish(output);
    */
  }

private:
  ros::NodeHandle nh;
  // 发布坐标变换
  tf::TransformBroadcaster br;
  // 发布landmarks点云
  ros::Publisher pclpub;
  ros::Publisher pclmappub;
  pcl::PointCloud<pcl::PointXYZ> globalmap;
  // 发布历史轨迹
  ros::Publisher pathpub;
  nav_msgs::Path path;  
};

// 作用：1、启动okvis核心组件
//      2、ROS的消息回调函数
//      3、okvis后端的注册函数
class ROSVIO
{
public:
  ROSVIO(okvis::VioParameters &parameters)
	:it_(nh_),
	 okvis_estimator(parameters)	 
  {
    // 初始化sub订阅者
    imagesub = it_.subscribe("cam0/image_raw",1,&ROSVIO::imageCallback,this);
    imusub = nh_.subscribe("imu0",100,&ROSVIO::imuCallback,this);
    
    //设置了阻塞，也就是说图片和IMU数据要一帧接着一帧处理，不能跳过
    okvis_estimator.setBlocking(true);
  }
  
  ~ROSVIO(){}
  
  // 图片回调函数
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    okvis::Time imagestamp(msg->header.stamp.sec,msg->header.stamp.nsec);
    
    //ROS_INFO("I heard IMAGE!");
    
    okvis_estimator.addImage( imagestamp , 0, image );
    
    //okvis_estimator.display();
    //poseviewer.display();
  }
  
  // imu回调函数
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {

    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
    okvis::Time imustamp(msg->header.stamp.sec,msg->header.stamp.nsec);
    gyr[0] = msg->angular_velocity.x;
    gyr[1] = msg->angular_velocity.y;
    gyr[2] = msg->angular_velocity.z;
    acc[0] = msg->linear_acceleration.x;
    acc[1] = msg->linear_acceleration.y;
    acc[2] = msg->linear_acceleration.z;
    //std::cout<<"gyr :"<<gyr[0]<<std::endl;
    
    //ROS_INFO("I heard IMU!");
    
    // 按道理应该按下面这种方法来进行添加imu数据
    okvis_estimator.addImuMeasurement( imustamp , acc, gyr );
    

  }
  
  void display()
  {
    // realtime display 处理结果实时显示部分，使用的是visualizationLoop函数生成的图片
      okvis_estimator.display();
  }
  
  // 发布path和tf（没有相机和IMU的相对位姿）
  void SysStateCallback()
  {
    okvis_estimator.setStateCallback(std::bind(&ROSVIOPUB::displayCallback,&rosviopub,std::placeholders::_1, std::placeholders::_2));
  }
  
  // 发布landmark点云
  void LandmarkCallback()
  {
    okvis_estimator.setLandmarksCallback(std::bind(&ROSVIOPUB::landmarkCallback,&rosviopub,std::placeholders::_1,
						   std::placeholders::_2,std::placeholders::_3));
  }
  
  // 发布path和tf
  void SysFullStateExtrinsicCallback()
  {
    okvis_estimator.setFullStateCallbackWithExtrinsics(std::bind(&ROSVIOPUB::fullstatewithextrinsicCallback,&rosviopub,
         std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
  }
  
  // 绘制二维轨迹图
  void displayCallback()
  {
    okvis_estimator.setFullStateCallback(std::bind(&PoseViewer::publishFullStateAsCallback,&poseviewer,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
  }
  
private:
  // ROS相关变量
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber imagesub;
  ros::Subscriber imusub;
  ROSVIOPUB rosviopub;
  // 绘制轨迹图
  PoseViewer poseviewer;
  // Okvis Node
  // okvis视觉惯性里程计的核心模块
  okvis::ThreadedKFVio okvis_estimator;
  
  
};

// this is just a workbench. most of the stuff here will go into the Frontend class.
int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);

  ros::init(argc,argv,"okvis");

  // read configuration file 读取配置文件
  std::string configFilename(argv[1]);
  okvis::VioParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);
  
  // 构造VIO对象
  ROSVIO rosvio(parameters);
  
  // 向OKVIS后端注册回调函数
  // 发布path和tf
  rosvio.SysFullStateExtrinsicCallback();
  // 发布landmark点云
  rosvio.LandmarkCallback();
  // 绘制轨迹图
  // rosvio.displayCallback();
  
  // 为两个ROS的图片回调函数和IMU回调函数开启线程
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}

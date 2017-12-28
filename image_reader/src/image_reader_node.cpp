#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/iterator/iterator_concepts.hpp>
#include <thread>
#include <mutex>
#include <stdlib.h>
// 互斥锁
std::mutex pubmutex;

class ImagePub
{
public:
  ImagePub(int num):numCameras(num)
  {
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub; 
    for( int i = 0; i < numCameras; ++i )
    {
      std::string pubname("camera");
      pubname = pubname + std::to_string(i) + "/image";
      pub = it.advertise(pubname, 1);
      pubs.push_back(pub);
    }
  }
  ~ImagePub(){}
  void pub_image(std::string path,std::vector<std::string>::iterator const &cam_iterators,int i)
  {
  
    std::cout<<"Thread ID:"<<std::this_thread::get_id()<<std::endl;
    cv::Mat image = cv::imread(path + "/cam" + std::to_string(i) + "/data/" + *cam_iterators,CV_LOAD_IMAGE_GRAYSCALE);
    // 图片文件的名字就是时间戳
    std::string nanoseconds = cam_iterators->substr(cam_iterators->size() - 13, 9);
    std::string seconds = cam_iterators->substr(0, cam_iterators->size() - 13);
    //std::cout << nanoseconds << std::endl;
    std_msgs::Header header;
    header.stamp.sec = std::stoi(seconds);
    header.stamp.nsec = std::stoi(nanoseconds);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
    pubmutex.lock();
    pubs[i].publish(msg);
    pubmutex.unlock();
  }
  
private:
  int numCameras;
  ros::NodeHandle nh;
  std::vector<image_transport::Publisher> pubs;
};


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "image_reader");
  int numCameras = 1;
  ImagePub imagepub(numCameras);
  
  // 读取文件中的
  /*
  std::string path(argv[1]);  
  int num_camera_images = 0;
  
  std::vector < std::vector < std::string >> image_names(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    num_camera_images = 0;
    std::string folder(path + "/cam" + std::to_string(i) + "/data");

    for (auto it = boost::filesystem::directory_iterator(folder);
        it != boost::filesystem::directory_iterator(); it++) {
      if (!boost::filesystem::is_directory(it->path())) {  //we eliminate directories
        num_camera_images++;
        image_names.at(i).push_back(it->path().filename().string());
      } else {
        continue;
      }
    }

    if (num_camera_images == 0) {
      //ROS_ERROR("no images at: %s", folder);
      return 1;
    }

    //ROS_INFO( "No. cam %d images: %d ",i,num_camera_images);
    // the filenames are not going to be sorted. So do this here
    std::sort(image_names.at(i).begin(), image_names.at(i).end());
  }
  
  //定义了和相机数量相等的迭代器
  std::vector < std::vector < std::string > ::iterator > cam_iterators(numCameras);
  for (size_t i = 0; i < numCameras; ++i) 
  {
    cam_iterators.at(i) = image_names.at(i).begin();
  }
  
  // 构造线程vector
  std::vector<std::thread> camera_threads;
  
  while(true)
  {
    
    for (size_t i = 0; i < numCameras; ++i) {
      // check if at the end 检查图片是否全部处理完了
      if (cam_iterators[i] == image_names[i].end()) {
        std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
        cv::waitKey();
        return 0;
      }
      // 启动与摄像头数量相等的线程
      camera_threads.push_back(std::thread(std::mem_fn(&ImagePub::pub_image),imagepub,path,cam_iterators.at(i),i));
    }
    
    std::for_each(camera_threads.begin(),camera_threads.end(),std::mem_fn(&std::thread::join));
    camera_threads.clear();
    for (size_t i = 0; i < numCameras; ++i) 
    {
      cam_iterators[i]++;      
    }
  }
  */
  return 0;
}

#include <string>
#include <iostream>
#include <fstream>
#include <boost/concept_check.hpp>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>

// for hf link and transport
#include <transport.h>
#include <transport_serial.h>
#include <hf_link.h>
#include <hf_hw.h>

class ImuPub
{
public:
  ImuPub(std::string url = "serial:///dev/ttyUSB0",
	 std::string config_addr = "/home/ubuntu/projects/vioproject/src/imu_reader/config.txt",
	 double pub_freq = 200):hf_hw_(url, config_addr),imupub_freq_(pub_freq)
  {
    pub = nh.advertise<sensor_msgs::Imu>("imu_measurement",100);
  }
  ~ImuPub(){}
  void pub_imu(double const *acc,double const *gyr,double const &seconds,double const &nanoseconds)
  {
    std::cout<<"gyr:"<<gyr[0]<<" , "<<gyr[1]<<" , "<<gyr[2]<<std::endl;
    std::cout<<"acc:"<<acc[0]<<" , "<<acc[1]<<" , "<<acc[2]<<std::endl;
    std::cout<<"seconds:"<<seconds<<std::endl;
    std::cout<<"gyr:"<<gyr<<std::endl;
    sensor_msgs::Imu msg;
    msg.header.stamp.sec = seconds;
    msg.header.stamp.nsec = nanoseconds;
    msg.header.frame_id = "IMU_FRAME";
    msg.angular_velocity.x = gyr[0];
    msg.angular_velocity.y = gyr[1];
    msg.angular_velocity.z = gyr[2];
    msg.linear_acceleration.x = acc[0];
    msg.linear_acceleration.y = acc[1];
    msg.linear_acceleration.z = acc[2];
    pub.publish(msg);
  }
  
  void mainloop()
  {
    // 设置IMU数据的发布频率
    ros::Rate loop(imupub_freq_);
    int count = 0;
    ros::Time currentTime = ros::Time::now();
    while (ros::ok())
    {
        hf_hw_.checkHandshake();
        // Read IMU Data
        if (hf_hw_.updateCommand(READ_IMU_DATA,count))
        {
            std::cout<< "spend time is  "<< (ros::Time::now() - currentTime).toSec()<<std::endl;
            ros::Time currentTime = ros::Time::now();

            sensor_msgs::Imu imudata;
            
	    imudata.header.stamp = currentTime;
	    imudata.header.frame_id = "imu";
	    
            imudata.angular_velocity.x = hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_gyro.x;
            imudata.angular_velocity.y = hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_gyro.y;
            imudata.angular_velocity.z = hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_gyro.z;
	    
	    imudata.linear_acceleration.x = hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_acc.x;
            imudata.linear_acceleration.y = hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_acc.y;
            imudata.linear_acceleration.z = hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_acc.z;
	    
	    std::ofstream imufile("/home/ubuntu/projects/pythonscript/imu.txt",std::ios::app);//ios::app表示在原文件末尾追加
	    if(!imufile) return;
	    imufile<<imudata.linear_acceleration.x<<" "<< imudata.linear_acceleration.y<<" "<<imudata.linear_acceleration.z<<" ";
	    imufile<<imudata.angular_velocity.x <<" "<<imudata.angular_velocity.y<<" "<<imudata.angular_velocity.z<<std::endl;
	    imufile.close();

            pub.publish(imudata);
	    
            //For Debugging
            /***********************************************************************************************************************
	    //std::cerr<<"IMU Data(pitch): "<<hf_hw_.getRobotAbstract()->measure_imu_euler_angle.pitch<<std::endl;
            std::cerr<<"IMU Data(accx): "<<hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_acc.x<<std::endl;
            std::cerr<<"IMU Data(accy): "<<hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_acc.y<<std::endl;
            std::cerr<<"IMU Data(accz): "<<hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_acc.z<<std::endl;

            std::cerr<<"IMU Data(gyrox): "<<hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_gyro.x<<std::endl;
            std::cerr<<"IMU Data(gyroy): "<<hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_gyro.y<<std::endl;
            std::cerr<<"IMU Data(gyroz): "<<hf_hw_.getRobotAbstract()->measure_imu_original_data.measure_imu_gyro.z<<std::endl;
            ************************************************************************************************************************/
        }

        loop.sleep();
        count++;
    }
  }
private:
  // ROS
  ros::NodeHandle nh;
  ros::Publisher pub;
  // Comunication
  handsfree_hw::HF_HW hf_hw_;
  // 发布IMU数据的频率
  double imupub_freq_;
};
int main(int argc,char** argv)
{
  ros::init(argc,argv,"imu_reader");
  // 注释：
  // config文件代表的意思，第一列是指通讯操作的名称，如READ_GLOBAL_SPEED，指的就是读取全局速度
  //                    第二列是指这项操作开启还是关闭，1代表开启，0代表关闭
  //                    第三列是指这项操作的执行频率，pub_freq_除以这个数字就代表相应的频率，比如pub_freq=10，而这个数字是5,则说明频率是10*5/100=0.5Hz
  // double pub_freq = 100;
  if( argc == 4 )
  {
    ImuPub imupub(argv[1],argv[2],std::stod(argv[3]));
    imupub.mainloop();
  }
  else if( argc == 3 )
  {
    ImuPub imupub(argv[1],argv[2]);
    imupub.mainloop();
  }
  else if( argc == 2 )
  {
    ImuPub imupub(argv[1]);
    imupub.mainloop();
  }
  else
  {
    ImuPub imupub;
    imupub.mainloop();
  }
  
  // 读取文件中的IMU信息
  /*
  // open the IMU file 读取IMU信息
  std::string path(argv[1]);
  std::string line;
  std::ifstream imu_file(path + "/imu0/data.csv");
  
  if (!imu_file.good()) {
    std::cerr<<"IMU file wrong!"<<std::endl;
    return -1;
  }
  
  int number_of_lines = 0;
  while (std::getline(imu_file, line))
      ++number_of_lines;
  std::cout<< "No. IMU measurements: " << number_of_lines-1;
  if (number_of_lines - 1 <= 0) 
  {
    return -1;
  }
  // set reading position to second line
  imu_file.clear();
  imu_file.seekg(0, std::ios::beg);
  std::getline(imu_file, line);
  
  while(true)
  {
    if (!std::getline(imu_file, line)) 
    {
      std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
      std::getchar();
      return 0;
    }

    std::stringstream stream(line);
    std::string s;
    std::getline(stream, s, ',');
    std::string nanoseconds = s.substr(s.size() - 9, 9);
    std::string seconds = s.substr(0, s.size() - 9);
    
    double gyr[3];
    for (int j = 0; j < 3; ++j) 
    {
      std::getline(stream, s, ',');
      gyr[j] = std::stof(s);
    }
    
    double acc[3];
    for (int j = 0; j < 3; ++j) 
    {
      std::getline(stream, s, ',');
      acc[j] = std::stof(s);
    }
    // 发布imu数据
    imupub.pub_imu(acc,gyr,std::stod(seconds),std::stod(nanoseconds));
    
  }
  */
  return 0;
}
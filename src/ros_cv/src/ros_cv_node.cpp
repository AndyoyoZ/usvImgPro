#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdio.h>

// #include <xx_msgs/Flag.h>

// static bool gainControl_flag = false;
// void  gainControlCB(const xx_msgs::Flag::ConstPtr& msg)
// {
//    std::string control_msg = msg->flag;
//    if(control_msg == "3dlidar stop,cv start")
//    {
//       ROS_INFO("ImagePro gain control.");
//       gainControl_flag = true;
//    }
// }

//读取yaml参数
template<typename T>
T getParam(const std::string& name,const T& defaultValue) //This name must be namespace+parameter_name
{
    T v;
    if(ros::param::get(name,v)) //get parameter by name depend on ROS.
    {
        ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        return v;
    }
    else 
        ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
    return defaultValue; //if the parameter haven't been set,it's value will return defaultValue.
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  // ros::Subscriber controlFlag_sub = nh.subscribe<xx_msgs::Flag>("flag_3dlidar_to_cv",1,gainControlCB); 
  std::string video_name = getParam<std::string>("ros_cv/videoname","rtsp://admin:txz123456@192.168.1.66:554/mjpeg/ch1/sub/av_stream");
  // std::string video_name = "rtsp://admin:txz123456@192.168.1.66:554/mjpeg/ch1/sub/av_stream";
  // std::string video_name = "/home/xx/andyoyo/usv_ImagePro/src/ros_cv/data/lib2.avi";
  int camera_index = 0;
  
  cv::VideoWriter writer("/home/xx/andyoyo/usv_ImagePro/src/ros_cv/data/write.avi",CV_FOURCC('M','J','P','G'),25.0,cv::Size(640,480));

  cv::VideoCapture cap(video_name);
  std::string img_name;
  if(!cap.isOpened())
  {
    std::cout<<"can not open camera!"<<std::endl;
    return -1;
  }
  cap.set(CV_CAP_PROP_FRAME_WIDTH ,640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  cv::Mat frame;
  ros::Rate loop_rate(30);
  
  while(nh.ok())
  {
    // if(gainControl_flag)//控制权
    {
      cap>>frame;
      if(frame.empty())
      {
        ROS_INFO("frame is empty!");
        std::cout<<"frame is empty!"<<std::endl;
        //TODO: 2018/12/26 是否该交出控制权？
        return -1;
      }
      else
      {
      //writer<<frame;//保存视频
      cv::imshow("camera frame",frame);
      if(cv::waitKey(10)==27)
      {
        ROS_INFO("ImagePro ESC...");
        std::cout<<"ESC!"<<std::endl;
        //TODO: 2018/12/26 是否该交出控制权？
        return -1;
      }
      //发布图像消息
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg(); 
      pub.publish(msg);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

}

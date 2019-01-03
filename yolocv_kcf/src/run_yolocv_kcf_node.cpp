//yolocv_kcf_node
//by:Andyoyo@swust
//data:2018.12.24
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
//kcf
#include "kcftracker.hpp"
//control_flag msg
#include <xx_msgs/Flag.h>

#define USING_MY_CFG //使用自己训练的配置文件
//#define USING_GPU  //使用Intel GPU
#define TRY_YOLO_COUNT 10 //yolo识别尝试次数，若识别失败，交出控制权

#define MIN_DIST  0.01
#define MAX_LINEAR_VEL  0.3
#define MAX_ANGULAR_VEL 0.5

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat& frame, std::vector<cv::Mat>& outs);
void postprocess(cv::Mat& frame, std::vector<cv::Mat>& outs,cv::Rect &resultBbox);
// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
//计算速度
void calcSpeed(cv::Rect &bBox,cv::Point &referencePoint,float &lineSpeed,float &angularVelocity);
//图像消息回调函数
void imageCB(const sensor_msgs::ImageConstPtr& msg);
//得到离目标点最近的box
bool getNearestBox(std::vector<int> &classIds,std::vector<cv::Rect> &bBoxes,std::vector<int> &indices,cv::Point &referencePoint,cv::Rect &nearestBox);
//run yolo3
void runyolo3(cv::Mat frame,cv::Rect &resultbox);
//run kcf
void runkcf(cv::Mat frame,cv::Rect initRect,cv::Rect &resultBbox);
//控制权消息回调函数
void  gainControlCB(const xx_msgs::Flag::ConstPtr& msg);




// YOLO参数
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 416;        // Width of network's input image
int inpHeight = 416;       // Height of network's input image
std::vector<std::string> classes;
cv::dnn::Net net;

//KCF参数
bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = false;
bool LAB = false;
// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
// Frame counter
int nFrames = 0;
//Init Rect
cv::Rect initRect(0,0,0,0);
//参考点
cv::Point referencePoint(320,480);
//初始化BBOX
cv::Rect yoloBbox(0,0,0,0);
cv::Rect kcfBbox(0,0,0,0);
//初始化标志位
bool yoloFindTarget = false;
bool kcfLost = true;
bool gainControl_flag = false;
int  trycount = TRY_YOLO_COUNT;
float lineSpeed=0; //线速度
float angularVelocity=0;//角速度

ros::Publisher vel_pub;//速度消息发布
ros::Publisher ctrl_pub;//控制权限消息发布

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
    ros::init(argc, argv, "yolocv_kcf");
    ros::NodeHandle nh,n1;

    vel_pub = n1.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",10);

    // Give the configuration and weight files for the model
    #ifdef USING_MY_CFG
    cv::String modelConfiguration = getParam<std::string>("yolocv_kcf/modelConfig","/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/mycfg/yolov3-tiny_xx.cfg");
    cv::String modelWeights = getParam<std::string>("yolocv_kcf/modelWeights","/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/mycfg/yolov3-tiny_xx_100000.weights");
    cv::String classesFile = getParam<std::string>("yolocv_kcf/modelNames","/home/xx/andyoyo/usv_ImagePro/src/yolo_cv/data/mycfg/voc_xx.names");
    //cv::String modelConfiguration = "/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/mycfg/yolov3-tiny_xx.cfg";
    //cv::String modelWeights = "/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/mycfg/yolov3-tiny_xx_100000.weights";
    //std::string classesFile = "/home/xx/andyoyo/usv_ImagePro/src/yolo_cv/data/mycfg/voc_xx.names"; 
    #else 
    cv::String modelConfiguration = getParam<std::string>("yolocv_kcf/modelConfig","/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/yolo3cfg/yolov3.cfg");
    cv::String modelWeights = getParam<std::string>("yolocv_kcf/modelWeights","/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/yolo3cfg/yolov3.weights");
    cv::String classesFile = getParam<std::string>("yolocv_kcf/modelNames","/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/yolo3cfg/coco.names");
    // cv::String modelConfiguration = "/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/yolo3cfg/yolov3.cfg";
    // cv::String modelWeights = "/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/yolo3cfg/yolov3.weights";
    // std::string classesFile = "/home/xx/andyoyo/usv_ImagePro/src/yolocv_kcf/data/yolo3cfg/coco.names"; 
    #endif
    
    std::ifstream classNamesFile(classesFile.c_str());
    if (classNamesFile.is_open())
    {
        std::string className = "";
        while (std::getline(classNamesFile, className))
            classes.push_back(className);
    }
    else{
        std::cout<<"can not open classNamesFile"<<std::endl;
    }
 
    // Load the network
    net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
    std::cout<<"Read Darknet..."<<std::endl;
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    
    #ifdef USING_GPU
    net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);//使用GPU 只能使用intel显卡
    #else
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);//使用CPU
    #endif

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/camera/image", 1, imageCB);//订阅camera
    ros::Subscriber controlFlag_sub = n1.subscribe<xx_msgs::Flag>("flag_nav_to_cv",1,gainControlCB);//订阅控制权限标志
    ctrl_pub=n1.advertise<xx_msgs::Flag>("flag_cv_to_nav",1);
    ros::Rate rate(10.0);

    while (ros::ok())
    {
        if(cv::waitKey(10)==27)
         {
            std::cout<<"ESC!"<<std::endl;
            return -1;
        }
        ros::spinOnce();
        rate.sleep();
    }

    
    return 0;
}


void  gainControlCB(const xx_msgs::Flag::ConstPtr& msg)
{
   std::string control_msg = msg->flag;
   std::cout<<"control_msg: "<<control_msg<<std::endl;
   if(control_msg == "nav stop,cv start")
   {
      ROS_INFO("ImagePro gain control.");
      gainControl_flag = true;
   }
   else if(control_msg == "nav start,cv stop")
   {
      ROS_INFO("ImagePro lost control.");
      gainControl_flag = false;
   }
   else
   {
      ROS_INFO("ImagePro lost control.");
      gainControl_flag = false;
   }
}

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
 if(gainControl_flag)
 {
    ROS_INFO("ImagePro gain control");
    cv::Mat frame = cv_bridge::toCvShare(msg,"bgr8")->image;
    cv::imshow("yolo_kcf_frame",frame);
    cv::waitKey(1);
    cv::Mat temp1;
    cv::Mat temp2;
    frame.copyTo(temp1);
    frame.copyTo(temp2);
    if(kcfLost)
    {
        
        lineSpeed = 0;
        angularVelocity = 0;
        
        if(trycount--)
        {
            ROS_INFO("run yolo %d times...",TRY_YOLO_COUNT-trycount);
            runyolo3(temp1,yoloBbox);
            std::cout<<"yoloBbox:"<<yoloBbox<<std::endl;
        }
        else //控制权交出
        {
            ROS_INFO("ImagePro lost control");
            gainControl_flag = false;
            trycount = TRY_YOLO_COUNT;
            xx_msgs::Flag flag_cv_to_nav;
	        flag_cv_to_nav.flag = "nav start,cv stop";
	        ctrl_pub.publish(flag_cv_to_nav);   //发布图像控制标志
        }

    }
    if(yoloFindTarget)
    {
        trycount = TRY_YOLO_COUNT;
        //控制权夺回
        ROS_INFO("run kcf...");
        runkcf(temp2,yoloBbox,kcfBbox);
    }
    	//发布速度信息
	geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = lineSpeed;
    vel_msg.angular.z =angularVelocity;
    vel_pub.publish(vel_msg);
 } 
 else{
     //ROS_INFO("Waiting control...");
 }       
}

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
         
        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();
         
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat& frame, std::vector<cv::Mat>& outs,cv::Rect &resultBbox)
{  
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes; 
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                 
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }

    
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);//抑制弱的重叠边界框
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
    
    cv::Rect nearestBox(0,0,0,0);

    if(getNearestBox(classIds,boxes,indices,referencePoint,nearestBox))//得到最近的bounding box
    {
        ROS_INFO("YOLO find bbox.");
        resultBbox = nearestBox;
        yoloFindTarget = true; 
    }
    else
    {
        ROS_INFO("YOLO not find bbox.");
        resultBbox = cv::Rect(0,0,0,0);
        yoloFindTarget = false;
    }
}

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255));
     
    //Get the label for the class name and its confidence
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    else
    {
        std::cout<<"classes is empty..."<<std::endl;
    }
     
    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
}

//Calculate line speed and angular velocity
void calcSpeed(cv::Rect &bBox,cv::Point &referencePoint,float &lineSpeed,float &angularVelocity)
{
    cv::Point bBoxCenetr = cv::Point(bBox.x+bBox.width/2,bBox.y+bBox.height/2);
    float offset_x = referencePoint.x - bBoxCenetr.x;
    float offset_y = referencePoint.y - bBoxCenetr.y;
    float dist_to_ref = 0.0008*sqrt(pow(offset_x,2)+pow(offset_y,2));
    float angle_to_ref = 0.5*atan(offset_x/offset_y);
    if(offset_y > 0 && dist_to_ref > MIN_DIST)//bounding box在参考点前方
    {
        lineSpeed = MAX_LINEAR_VEL < dist_to_ref ? MAX_LINEAR_VEL : dist_to_ref;//0~0.7
        angularVelocity= -MAX_ANGULAR_VEL > angle_to_ref ? -MAX_ANGULAR_VEL : MAX_ANGULAR_VEL < angle_to_ref ? MAX_ANGULAR_VEL : angle_to_ref;//-0.7~0.7 
    }
    else //停止
    {
        lineSpeed=0;
        angularVelocity=0;
    }
}

//得到离目标点最近的box
bool getNearestBox(std::vector<int> &classIds,std::vector<cv::Rect> &bBoxes,std::vector<int> &indices,cv::Point &referencePoint,cv::Rect &nearestBox)
{
    if(indices.size()>0)
    {
     int nearestIdx;
     bool init = true;
     bool discoverTarget = false;
     for(size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            //ROS_INFO("Determine if it is a target class.");
            if(classIds[idx]==0||classIds[idx]==1/*||classIds[idx]==41*/)//判断是否为目标类 0 person  41cup 
            {
                discoverTarget = true;
                if(init)
                {
                    nearestIdx = idx;
                    init = !init;
                }
                else
                {
                    cv::Rect box1 = bBoxes[nearestIdx];
                    cv::Rect box2 = bBoxes[idx];
                    cv::Point bBoxCenetr1 = cv::Point(box1.x+box1.width/2,box1.y+box1.height/2);
                    cv::Point bBoxCenetr2 = cv::Point(box2.x+box2.width/2,box2.y+box2.height/2);
                    int dist1 = abs(bBoxCenetr1.x-referencePoint.x)+abs(bBoxCenetr1.y-referencePoint.y);//街区距离
                    int dist2 = abs(bBoxCenetr2.x-referencePoint.x)+abs(bBoxCenetr2.y-referencePoint.y);//街区距离
                    if(dist1 > dist2)
                    {   
                        nearestIdx = idx;
                    }   
                }
 
            }

        }
        if(discoverTarget)
        {
            nearestBox = bBoxes[nearestIdx];
            return true;
        }
        else
        {
            // std::cout<<"No target found...\n";
            return false;
        }     
    }
    else
    {
        // std::cout<<"No box found...\n";
        return false;
    }
}


void runyolo3(cv::Mat frame,cv::Rect &resultbbox)
{
        // Create a 4D blob from a frame.
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);
     
    //Sets the input to the network
    net.setInput(blob);
     
    // Runs the forward pass to get output of the output layers
    std::vector<cv::Mat> outs;
    net.forward(outs, getOutputsNames(net));
     
    // Remove the bounding boxes with low confidence
    postprocess(frame, outs,resultbbox);
     
    // Put efficiency information. The function getPerfProfile returns the 
    // overall time for inference(t) and the timings for each of the layers(in layersTimes)
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    // std::string label = cv::format("Inference time for a frame : %.2f ms", t);
    std::string label = cv::format(" yolo %.2f fps", 1000.0/t);
    cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
     
    // Write the frame with the detection boxes
    cv::Mat detectedFrame;
    frame.convertTo(detectedFrame, CV_8U);
    //show detectedFrame
    cv::imshow("yoloDetectedFrame",detectedFrame);
    cv::waitKey(1);
}

void runkcf(cv::Mat frame,cv::Rect initRect,cv::Rect &resultBbox)
{

    double t=0,fps=0;
	char str[10];
 	t = (double)cv::getTickCount();
	// First frame, give the rectangle to the tracker
	if (nFrames == 0||kcfLost) 
		{
			if(initRect.width>0&&initRect.height>0)
			{
				ROS_INFO("kcf init...");
				kcfLost = false;
				tracker.init( initRect, frame );
				cv::rectangle(  frame, 
							cv::Point( initRect.x, initRect.y ), 
							cv::Point( initRect.x+initRect.width, initRect.y+initRect.height),
							cv::Scalar( 0, 255, 255 ), 1, 8 );
				initRect = cv::Rect(0,0,0,0);
			}
			else
			{
				ROS_INFO("no kcf init Rect");
			}
			
		}
		// Update
		else
		{
			ROS_INFO("kcf update...");
			if(tracker.update(frame,resultBbox))
			{
				cv::rectangle(  frame, 
						cv::Point( resultBbox.x, resultBbox.y ), 
						cv::Point( resultBbox.x+resultBbox.width, 
						resultBbox.y+resultBbox.height), 
						cv::Scalar( 0, 255, 255 ), 1, 8 );
				
				calcSpeed(resultBbox,referencePoint,lineSpeed,angularVelocity);
                ROS_INFO("lineSpeed: %f  angularVelocity: %f",lineSpeed,angularVelocity);
				cv::line(frame,referencePoint,cv::Point(resultBbox.x+resultBbox.width/2,resultBbox.y+resultBbox.height/2),cv::Scalar(255,0,0),2); 			
			}
			else
			{
				ROS_INFO("kcf update failed...");
				kcfLost = true;
			}
		}

	//计算运行时间
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	fps = 1.0 / t;
	sprintf(str, "%.2f fps", fps);
	std::string fpsString("kcf: ");
	fpsString+=str;
	// 将帧率信息写在输出帧上
    cv::putText(frame,               // 图像矩阵
            fpsString,           // string型文字内容
            cv::Point(5, 20),        // 文字坐标，以左下角为原点
            cv::FONT_HERSHEY_SIMPLEX,// 字体类型
            0.5,                 // 字体大小
            cv::Scalar(0, 0, 255));  // 字体颜色
	nFrames++;
	if (!SILENT)
	{
		cv::imshow("kcf tracker", frame);
		cv::waitKey(1);
	}

}
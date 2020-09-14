// Header file for the class
#include "YOLONode.h"

#include <unistd.h>
#include <std_msgs/String.h>
#include <yolo/Detections.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>

#include "YOLO.h"

#define WEIGHTS_NAME "run.weights"
#define CONFIG_NAME "run.cfg"
// #define IMAGE_TOPIC "/front_camera/image_raw"
#define IMAGE_TOPIC "/camera_front/image_raw"

#define DEBUG

const static float CAMERA_HORIZ_FOV = 0.80;
const static float HORIZ_OFFSET = CAMERA_HORIZ_FOV / 2;
const static float CAMERA_VERT_FOV = 0.78; //horiz FOV * height / width; 1.04 * 480/640
const static float VERT_OFFSET = CAMERA_VERT_FOV / 2;



// Namespace matches ROS package name
namespace yolo {

    // Constructor with global and private node handle arguments
    YOLONode::YOLONode(ros::NodeHandle& n, ros::NodeHandle& pn) //: tf_listener(tf_buffer)
    {
		status_publisher = n.advertise<std_msgs::String>("status", 1);		
        detections_publisher = n.advertise<yolo::Detections>("yolo/detections", 1);
        detected_image_publisher = n.advertise<sensor_msgs::Image>("yolo/detected_image", 1);
		std_msgs::String message; 
		message.data = "Starting up...";
		status_publisher.publish(message);
        mError = false;
       
		if(InitializeYOLO())
		{			
			message.data = "YOLO successfully initialized";
		}
		else
		{		
			message.data = "YOLO failed to initialize";
		}
		status_publisher.publish(message);
		
		image_subscriber = n.subscribe(IMAGE_TOPIC, 1, &YOLONode::ImageTopicCallback, this);
		
    }
    YOLONode::~YOLONode()
    {
        delete mYOLO;
    }

    bool YOLONode::InitializeYOLO()
    {
		printf("\nYOLOOOOO!!!\n");
		      
		mYOLO = new YOLO();
        
        char buff[512];
        memset(buff, 0, sizeof(buff));
        if (getcwd(buff, sizeof(buff)) == NULL)
        {
            mLastError = "Unable to determine working directory";
            return false;
        }

        std::string working_directory = buff;

        std::string weights_path = working_directory;
        weights_path += "/darknet/";
        weights_path += WEIGHTS_NAME;
        mYOLO->SetWeightsPath(weights_path);

        std::string config_path = working_directory;
        config_path += "/darknet/";
        config_path += CONFIG_NAME;
        mYOLO->SetConfigurationPath(config_path);

        mYOLO->SetDetectionThreshold(0.65);
        mYOLO->SetNMSThreshold(0.1);

#ifdef DEBUG
        mYOLO->DrawDetections(true);
#else
        mYOLO->DrawDetections(false);
#endif

        if(!mYOLO->Initialize())
        {
            mLastError = "Failed to initialize YOLO. Are these paths correct?\nWeights:\t";
            mLastError += weights_path;
            mLastError += "\nConfig:\t";
            mLastError += config_path;
            return false;
        }
		printf("\nInit Finished\n");
        return true;
    }

	void YOLONode::ImageTopicCallback(const sensor_msgs::ImageConstPtr& image_msg)
	{   
        if(mError)
        {
            printf("Error Detected, frame discarded\n");
            return;  
        }
       
        cv_bridge::CvImagePtr cv_image;
        try 
        {
            cv_image = cv_bridge::toCvCopy(image_msg, "rgb8");
        } 
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        IplImage* ipl_image = new IplImage(cv_image->image);

        /*geometry_msgs::TransformStamped transform;
        try{
            tf_buffer.lookupTransform("front_camera", "base_footprint", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }*/
        
        std::vector<Detection_t> detections = mYOLO->Detect(ipl_image);
        yolo::Detections detections_msg;
        yolo::Detection detection;
        for(int i = 0; i < detections.size(); i++)
        {
            
            detection.class_id = detections[i].class_id;
            detection.angle_left = (detections[i].x * CAMERA_HORIZ_FOV) - HORIZ_OFFSET;
            detection.angle_right = detection.angle_left + (detections[i].w * CAMERA_HORIZ_FOV);
            detection.angle_top = (detections[i].y * CAMERA_VERT_FOV) - VERT_OFFSET;
            detection.angle_bottom = detection.angle_top + (detections[i].h * CAMERA_VERT_FOV);
            detection.confidence = detections[i].confidence;  
            detections_msg.detections.push_back(detection);
        }
                
        detections_publisher.publish(detections_msg);

#ifdef DEBUG
        cv_bridge::CvImage cv_detected_image;
        IplImage* ipl_detected_image = mYOLO->GetDetectedImage();
        cv::Mat mat_detected_image = cv::cvarrToMat(ipl_detected_image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", mat_detected_image).toImageMsg();
        detected_image_publisher.publish(msg);
#endif
        
        delete ipl_image;
    }


}

#ifndef FLANNMATCH_H
#define FLANNMATCH_H

// ROS HEADERS
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <string>
#include <sstream>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class FlannMatching
{
public:
	FlannMatching(ros::NodeHandle nh_);     //constructor
	~FlannMatching();       //destructor
    void setDetector(std::string Det);
    void setExtractor(std::string Ex);
    void setMatcher(std::string Match);
    void setPath(std::string P);
    //void setKnn(bool kN);
    void setK(int _K);
    void setDistCheck(float Dist);
    std::string getDetector();
    std::string getExtractor();
    std::string getMatcher();
    std::string getPath();
    //bool getKnn();
    int getK();
    float getDistCheck();
	
protected:
	void imageCB(const sensor_msgs::ImageConstPtr& msg);        //receives the images feed and converts it
	//void guiCB(const tiago_opencv_tutorial::valueMatrixConstPtr& msg);
	//void homography(std::vector<cv::KeyPoint> aruco_, std::vector<cv::KeyPoint> feed_, std::vector<cv::DMatch> match, cv::Mat aruco_img, cv::Mat matches_mat);
	void Matcher(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out);      //keypoints, descriptors and matching computation

	image_transport::ImageTransport _imageTransport;
	image_transport::Subscriber image_sub;
	//ros::Subscriber  gui_sub;

	std::string descriptor_extractor, feature_detector, descriptor_matcher, path;
	//bool knn;
	int k;
	float dist_check;
    const std::string win = "Matcher";
};

#endif
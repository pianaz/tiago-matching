#include <tiagomatch/Flann.h>

FlannMatching::FlannMatching(ros::NodeHandle nh_): _imageTransport(nh_)
{
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &FlannMatching::imageCB, this, image_transport::TransportHints("compressed"));
//gui_sub = nh_.subscribe("/tiago_opencv_tutorial/flann_matching_gui", 1, &FlannMatching::guiCB, this);
	
	cv::namedWindow(win, CV_WINDOW_KEEPRATIO);
	
	feature_detector = "SURF";
	descriptor_extractor = "SURF";
	descriptor_matcher = "FlannBased";
	//knn = false;
	k = 2;
	dist_check = 0.6;
	path = "/home/pianaz/tiago_public_ws/src/tiagomatch/src/TIAGO.jpg";
}

FlannMatching::~FlannMatching()
{
	cv::destroyAllWindows();
}

void FlannMatching::setDetector(std::string Det)
{
    feature_detector = Det;
}

void FlannMatching::setExtractor(std::string Ex)
{
    descriptor_extractor = Ex;
}

void FlannMatching::setMatcher(std::string Match)
{
    descriptor_matcher = Match;
}

void FlannMatching::setPath(std::string P)
{
    path = P;
}

void FlannMatching::setK(int _K)
{
    k = _K;
}

void FlannMatching::setDistCheck(float Dist)
{
    dist_check = Dist;
}

std::string FlannMatching::getDetector()
{
    return feature_detector;
}

std::string FlannMatching::getExtractor()
{
    return descriptor_extractor;
}

std::string FlannMatching::getMatcher()
{
    return descriptor_matcher;
}

std::string FlannMatching::getPath()
{
    return path;
}

int FlannMatching::getK()
{
    return k;
}

float FlannMatching::getDistCheck()
{
    return dist_check;
}

/*
void FlannMatching::guiCB(const tiago_opencv_tutorial::valueMatrixConstPtr& msg)
{
	if(msg->header.frame_id == "dist")
		dist_check = msg->value;
	else if(msg->header.frame_id == "k")
		k = msg->value;
	else if(msg->header.frame_id == "k")
		k = msg->value;
	else if(msg->header.frame_id == "dist_check")
		dist_check = msg->value;
	else if(msg->header.frame_id == "path")
		path = msg->option;
	else if(msg->header.frame_id == "Keypoints")
		knn = msg->tick;
	else if(msg->header.frame_id == "feature_choice")
		feature_gui = msg->option;
	else if(msg->header.frame_id == "extracter_choice")
		extractor_gui = msg->option;
	else if(msg->header.frame_id == "matcher_choice")
		matcher_gui = msg->option;
}
*/

void FlannMatching::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat img, new_mat;
	cv_bridge::CvImagePtr cvPtr;

//copy the sensor (camera) message in a cv bridge pointer
	try
	{ 
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

//copy the the content of the pointer in a mat object
	cvPtr->image.copyTo(img);

//read the static image
	cv::Mat img_stat = cv::imread(path);
	if(!img_stat.data)
		ROS_INFO("NO DATA FROM STATIC IMAGE");
	if(!img.data)
		ROS_INFO("NO DATA FROM IMAGE FEED");

//call the matcher function
	this->Matcher(img, img_stat, new_mat);
//show the result
	cv::imshow(win, new_mat);
	cv::waitKey(1);
}

void FlannMatching::Matcher(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out)
{
	cv::initModule_nonfree();

	cv::Mat desc_feed, desc_static;
	cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(feature_detector);

	std::vector<cv::KeyPoint> vec_feed, vec_static;

	detector->detect(in_feed, vec_feed);
	detector->detect(in_static, vec_static);

	cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(descriptor_extractor);
    extractor->compute(in_feed, vec_feed, desc_feed);
	extractor->compute(in_static, vec_static, desc_static);

	if(desc_feed.type() != CV_32F || desc_static.type() != CV_32F)
	{
		desc_feed.convertTo(desc_feed, CV_32F);
		desc_static.convertTo(desc_static, CV_32F);
	}

/*
	if(matcher_gui == "BruteForce-Hamming" || matcher_gui == "BruteForce-Hamming(2)")
	{
		desc_feed.convertTo(desc_feed, CV_8U);
		desc_static.convertTo(desc_static, CV_8U);
	}
*/

	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(descriptor_matcher);
    std::vector<std::vector<cv::DMatch>> matches;
	matcher->knnMatch(desc_feed, desc_static, matches, k);

	std::vector<cv::DMatch> good_matches;

	for(const auto m : matches)
		if(m[0].distance < m[1].distance * dist_check)
			good_matches.push_back(m[0]);

	cv::drawMatches(in_feed, vec_feed, in_static, vec_static, good_matches, out);

//display the number of matches and good matches found along with the image
	cv::Point org(out.cols/2, out.rows/2); //da cambiare

	cv::Scalar color (1,255,1);

	int Msize = matches.size();
	int GMsize = good_matches.size();

	std::stringstream mm;

	mm << Msize << "matches;" << GMsize << "good matches";

	std::string m = mm.str();

	cv::putText( out, m, org, CV_FONT_HERSHEY_COMPLEX, 3,
           color, 5, 8 );

	
}

/*
void FlannMatching::homography(std::vector<cv::KeyPoint> aruco_, std::vector<cv::KeyPoint> feed_, std::vector<cv::DMatch> match_vector, cv::Mat aruco_img, cv::Mat matches_mat)
{
	std::vector<cv::Point2f> aruco_2f, feed_2f;
	
	for(int i = 0; i<match_vector.size(); ++i)
	{
		aruco_2f.push_back(aruco_[match_vector[i].queryIdx].pt);
		feed_2f.push_back(feed_[match_vector[i].trainIdx].pt);
	}

	cv::Mat h_mat = cv::findHomography(aruco_2f, feed_2f, CV_RANSAC );

	std::vector<cv::Point2f>aruco_corners(4), feed_corners(4);
	aruco_corners[0] = cv::Point2f(0,0);
	aruco_corners[1] = cv::Point2f(matches_mat.cols, 0);
	aruco_corners[2] = cv::Point2f(matches_mat.cols, matches_mat.rows);
	aruco_corners[3] = cv::Point2f(0, matches_mat.rows);

	cv::perspectiveTransform(aruco_corners, feed_corners, h_mat);

	cv::line(matches_mat, feed_corners[0] + cv::Point2f(aruco_img.cols, 0), feed_corners[1] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);
	cv::line(matches_mat, feed_corners[1] + cv::Point2f(aruco_img.cols, 0), feed_corners[2] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);
	cv::line(matches_mat, feed_corners[2] + cv::Point2f(aruco_img.cols, 0), feed_corners[3] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);
	cv::line(matches_mat, feed_corners[3] + cv::Point2f(aruco_img.cols, 0), feed_corners[0] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);
}
*/
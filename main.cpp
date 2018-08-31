
#include <tiagomatch/Flann.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "FlannMatching");

	ros::NodeHandle nh;

	FlannMatching fm(nh);

	ros::spin();
}

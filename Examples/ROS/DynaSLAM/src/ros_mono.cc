/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<vector>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ctime>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <signal.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"../../../include/System.h"
#include "../../../include/Geometry.h"
#include "../../../include/MaskNet.h"

#include<ros/ros.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <DenseInput.h>
using namespace std;
using namespace cv;

void mySigintHandler(int sig) {
	ros::shutdown();
}

ros::Publisher pose_pub; 
ros::Publisher pub_dense;
double old_max;
double old_min;
bool init = false;

string outputPath;

class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	ORB_SLAM2::System* mpSLAM;
	DynaSLAM::SegmentDynObject* MaskNet;
	cv::Mat kernel;
};


vector<cv::Mat> gyroMatrices;
int gyroFrameCounter;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mono");
	ros::start();

	if(argc != 6)
	{
		cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings path_to_sequence path_to_masks output_path" << endl;        
		ros::shutdown();
		return 1;
	}    

	outputPath = argv[5];
	mkdir(argv[5], S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	mkdir((outputPath + "/viewMatrix").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	cerr << endl << "Initialization" << endl;
	// Load gyroscope matrix data here.
	
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
        cerr << endl << "Done!" << endl;

	ImageGrabber igb(&SLAM);

	cout << "Loading Mask R-CNN. This could take a while..." << endl;
	igb.MaskNet = new DynaSLAM::SegmentDynObject();
	cout << "Mask R-CNN loaded!" << endl;

	int dilation_size = 15;
    	igb.kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                        cv::Point( dilation_size, dilation_size ) );

	ros::NodeHandle nodeHandler;
	ros::Subscriber sub = nodeHandler.subscribe("/camera/rgb/image_color", 1000, &ImageGrabber::GrabImage, &igb);
	//ros::Subscriber sub = nodeHandler.subscribe("/status", 1000, mySigintHandler);
	pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/camera_pose",1000);
	pub_dense = nodeHandler.advertise<svo_msgs::DenseInput>("/ORB/DenseInput",1000);

	signal(SIGINT, mySigintHandler);
	
	ros::spin();

    // Stop all threads
	SLAM.Shutdown();

    // Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	ros::shutdown();

	return 0;
}




int id = 0;
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    clock_t start, end;
    double cpu_time_used;
    start = clock();
    // Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_bridge::CvImageConstPtr cv_ptr_rgb;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::MONO8);
		cv_ptr_rgb = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	stringstream ss;
	string str;
	ss << id << ".txt";
	ss >> str; 

	cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
        cv::Mat maskRCNN;
        maskRCNN = MaskNet->GetSegmentation(im,string(argv[4]),str); //0 background y 1 foreground
        cv::Mat maskRCNNdil = maskRCNN.clone();
        cv::dilate(maskRCNN,maskRCNNdil, kernel);
        mask = mask - maskRCNNdil;
	

	cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image, mask, cv_ptr->header.stamp.toSec());
	if (pose.empty())
		return;
	
    end = clock();
    cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM("exe time: " << cpu_time_used << "\n");
}



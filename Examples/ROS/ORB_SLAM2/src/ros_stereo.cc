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
#include<algorithm>
#include<fstream>
#include<chrono>

#include<sys/socket.h>	//socket
#include<arpa/inet.h>	//inet_addr
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){do_rectify=true;}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};
void send_signal(int socket,char message[])
{
    if (send(socket,message,sizeof(message),0))
    {
        std::cout << "we sent a replay: "<<message<< std::endl;
    }else
    {
        std::cout << "coudlnt send a replay"<< std::endl;
    }
}
int connect_to_socket(int port){
	struct sockaddr_in server;
	int sock;
	//Create socket
	sock = socket(AF_INET , SOCK_STREAM , 0);
	if (sock == -1)
	{
		printf("Could not create socket");
	}
	puts("Socket created");
	
	server.sin_addr.s_addr = inet_addr("127.0.0.1");
	server.sin_family = AF_INET;
	server.sin_port = htons(port);
    while (1)
    {
        if (connect(sock , (struct sockaddr *)&server , sizeof(server)) >= 0)
        {
            std::cout <<"connection successed";
            return sock;
        }
    }
    
	//Connect to remote server
	
    return sock;
}
void saveCurrentPosition(cv::Mat pose){

	cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
	cv::Mat twc = -Rwc*pose.rowRange(0,3).col(3);
	std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
	std::ofstream currentPositionFile;
	currentPositionFile.open("/tmp/current_position.csv",std::ofstream::out | std::ofstream::trunc);
	currentPositionFile << twc.at<float>(0) << "," << twc.at<float>(2) << "," << twc.at<float>(1) << ","
	<< q[0] << "," << q[1] << "," << q[2] << "," << q[3] << std::endl;
	currentPositionFile.close();
}
bool stopThread = false;

void saveMap( int fileNumber,ORB_SLAM2::System* SLAM ){
	std::ofstream pointData;
    pointData.open("/tmp/pointData"+ std::to_string(fileNumber) + ".csv");
	for(auto mapPoint : SLAM->GetMap()->GetAllMapPoints()){
        if (mapPoint != NULL)
        {
            auto frame = mapPoint->GetReferenceKeyFrame();
            auto pose = frame->GetPoseInverse();
            auto Rwc = pose.rowRange(0,3).colRange(0,3);
            std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
            cv::Mat twc = mapPoint->GetWorldPos();
            pointData << twc.at<float>(0) << "," << twc.at<float>(2) << "," << twc.at<float>(1) << ","
            << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << std::endl;
        }
	}
	pointData.close();
	std::cout<<"map saved" <<std::endl;
}
void mapSaver(ORB_SLAM2::System* SLAM ){
    int mapCounter = 0;
    int socket = connect_to_socket(1234);
    char server_reply[2000];
    char message[2000];
    std::string command = "";
    while(1){
        if (stopThread)
        {
            break;
        }
        
        if(recv(socket , server_reply , 2000 , 0) >= 0)
		{
            command = std::string(server_reply);
            if (command.find("exit") != std::string::npos)
            {
                break;
            }
            else if (command.find("save map") != std::string::npos){
                
                saveMap(mapCounter++,SLAM );
                std::string tmp = "map saved";
                std::strcpy(message, tmp.c_str());
                send_signal(socket,message);
            }
		}
    }
    close(socket);
    std::cout << "we closed socket"<< std::endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);
    std::thread thread_obj(mapSaver,&SLAM);

        // Load settings related to stereo calibration
    float K_ldata[9] ={549.5046467375682, 0.0, 324.8875430810698, 0.0, 545.5571194080444, 226.6473597465127, 0.0, 0.0, 1.0};
    cv::Mat K_l = cv::Mat::zeros(3, 3, CV_32F);
    for(int i =0; i<9;i++){
    K_l.at<float>(i) = K_ldata[i];
	}
	float K_rdata[9] ={550.985684274004, 0.0, 314.30783300961616, 0.0, 548.47443248151, 253.59695839417765, 0.0, 0.0, 1.0};
        cv::Mat K_r = cv::Mat::zeros(3, 3, CV_32F);
        for(int i =0; i<9;i++){
	    K_r.at<float>(i) = K_rdata[i];
	}  
	float P_ldata[12] ={650.9102714731106, 0.0, 238.38889122009277, 0.0, 0.0, 650.9102714731106, 238.6435832977295, 0.0, 0.0, 0.0, 1.0, 0.0};
        cv::Mat P_l = cv::Mat::zeros(3, 4, CV_32F);
        for(int i =0; i<12;i++){
	    P_l.at<float>(i) = P_ldata[i];
	}  
	float P_rdata[12] ={650.9102714731106, 0.0, 238.38889122009277, -3801.759550053265, 0.0, 650.9102714731106, 238.6435832977295, 0.0, 0.0, 0.0, 1.0, 0.0};
        cv::Mat P_r = cv::Mat::zeros(3, 4, CV_32F);
        for(int i =0; i<12;i++){
	    P_r.at<float>(i) = P_rdata[i];
	} 
	float R_ldata[9] ={0.9836457952297356, -0.0736378609607167, 0.1643727926391438, 0.07714958053634623, 0.996905560325608, -0.015074681255383315, -0.1627540836658996, 0.027509438834984093, 0.9862830927401409};
        cv::Mat R_l = cv::Mat::zeros(3, 3, CV_32F);
        for(int i =0; i<9;i++){
	    R_l.at<float>(i) = R_ldata[i];
	}
	float R_rdata[9] ={0.9972550771222964, -0.031142786311299065, 0.06717468283937914, 0.02969849976969111, 0.9993080322028414, 0.022393210719206295, -0.06782558709832207, -0.020336755779963515, 0.9974899027554684};
        cv::Mat R_r = cv::Mat::zeros(3, 3, CV_32F);
        for(int i =0; i<9;i++){
	    R_r.at<float>(i) = R_rdata[i];
	}  
	float D_ldata[5] ={-0.07157265208574681, 0.10169777861568328, 2.078766642164336e-05, 0.00290716486350998, 0.0};
        cv::Mat D_l = cv::Mat::zeros(1, 5, CV_32F);
        for(int i =0; i<5;i++){
	    D_l.at<float>(i) = D_ldata[i];
	}
	float D_rdata[5] ={-0.07907708176724568, 0.11524155110288563, 0.005205345105151602, 0.0015283031747578733, 0.0};
        cv::Mat D_r = cv::Mat::zeros(1, 5, CV_32F);
        for(int i =0; i<5;i++){
	    D_r.at<float>(i) = D_rdata[i];
	}
        float rows_l = 480;
        float cols_l = 640;
        float rows_r = 480;
        float cols_r = 640;

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/left/usb_cam/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/right/usb_cam/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();
    saveMap(34234,&SLAM);
    // Stop all threads
    stopThread = true;
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat imLeft, imRight;
    cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
    cv::Mat pose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    if(!pose.empty()){
       saveCurrentPosition(pose);
    }

}



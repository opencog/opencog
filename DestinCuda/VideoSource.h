/*
 * VideoSource.h
 *
 *  Created on: Nov 3, 2011
 *      Author: Ted Sanders
 */

#ifndef VIDEOSOURCE_H_
#define VIDEOSOURCE_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;
class VideoSource {
private:

	cv::VideoCapture * cap;
	cv::Mat original_frame;
	cv::Mat rescaled_frame;
	cv::Mat greyscaled_frame;
	cv::Size original_size;
	cv::Size target_size;
	string window_title;


public:
	/**
	 * use_device - if true then will find the default device i.e. webcam as input
	 * and will ignore video_file, otherwise use the video at video_file as input
	 *
	 * video_file - video file to use as input when use_device is false
	 *
	 * dev_no - if use_device is true, specifies the device number to use, defaults to 0
	 * which should bring up the default device.
	 */
	VideoSource(bool use_device, std::string video_file, int dev_no = 0)
	:target_size(512, 512), window_title("edges"){
		if(use_device){
			cap = new cv::VideoCapture(dev_no);
		}else{
			cap = new cv::VideoCapture(video_file);
		}
	}

	bool isOpened(){
		return cap->isOpened();
	}
	void setSize(int width, int height){
		target_size = cv::Size(width, height);
	}

	~VideoSource(){
		delete cap;
	}

	/**
	 * Shows the output of the video or webcam to the screen in a window
	 */
	//see http://opencv.willowgarage.com/documentation/cpp/user_interface.html#cv-namedwindow
	void showVideo(){//don't know how to unshow it yet
		cv::namedWindow(window_title);
	}
	bool grab();
};

#endif /* VIDEOSOURCE_H_ */

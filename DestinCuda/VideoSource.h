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
#include <stdexcept>


using namespace std;


class VideoSource {
private:

	//webcam output is resized to this size:
	cv::Size target_size;

	//pointer to the video source
	cv::VideoCapture * cap;

	//video as originally comes out of the webcam
	cv::Mat original_frame;
	//video after it has been resized to the target_size
	cv::Mat rescaled_frame;
	//video after it has been resize and converted to greyscale
	cv::Mat greyscaled_frame;

	//the greyscaled_frame converted to a float array to be fed to DeSTIN
	float * float_frame;

	//the original size as it came out of the video device
	cv::Size original_size;

	//title on the window which displays the video output
#define DESTIN_VIDEO_WINDOW_TITLE  "DeSTIN Input Video"

	bool edge_detection; //if true shows video in edge detection mode ( shows image outlines)
	/**
	 * convert - converts from an OpenCV Mat greyscal 8bit uchar image into
	 * a float array where each element is normalized to 0 to 1.0, with 1,0 being black.
	 *
	 * Assumes out points to a preallocated float array big enough to hold the
	 * converted image ( should be of size target_size)
	 */
	void convert(cv::Mat & in, float * out) {
		if(in.channels()!=1){
			throw runtime_error("Excepted a grayscale image with one channel.");
		}
		if(in.depth()!=CV_8U){
			throw runtime_error("Expected image to have bit depth of 8bits unsigned integers ( CV_8U )");
		}
		cv::Point p(0, 0);
		int i = 0 ;
		for (p.y = 0; p.y < in.rows; p.y++) {
			for (p.x = 0; p.x < in.cols; p.x++) {
				//i = frame.at<uchar>(p);
				//use something like frame.at<Vec3b>(p)[channel] in case of trying to support color images.
				//There would be 3 channels for a color image (one for each of r, g, b)
				out[i] = (float)in.at<uchar>(p) / 255.0f;
				i++;
			}
		}
	}
public:
	/**
	 * use_device - if true then will find the default device i.e. webcam as input
	 * and will ignore video_file, otherwise use the video file at video_file path as input
	 *
	 * video_file - video file to use as input when use_device is false
	 *
	 * dev_no - if use_device is true, specifies the device number to use,
	 * a value of 0 should bring up the default device.
	 *
	 */
	VideoSource(bool use_device, std::string video_file, int dev_no = 0) :
		target_size(512, 512), edge_detection(false) {

		float_frame = new float[target_size.area()];
		stringstream mess;
		if (use_device) {
			cap = new cv::VideoCapture(dev_no);
		} else {
			cap = new cv::VideoCapture(video_file);
		}
		if(!cap->isOpened()){
			if(use_device){
				mess << "Could not open capturing device with device number " << dev_no << endl;
			}else{
				mess << "Could not open video file " << video_file << endl;
			}
			throw runtime_error(mess.str());
		}
	}

	bool isOpened() {
		return cap->isOpened();
	}

	/**
	 * setSize - rescales video output to this size
	 */
	void setSize(int width, int height) {
		if(target_size.width != width || target_size.height != height){
			delete [] float_frame;
			float_frame = new float[width * height];
		}
		target_size = cv::Size(width, height);
	}

	~VideoSource() {
		delete cap;
		delete [] float_frame;
	}


	/**
	 * Gets the pointer to the greyscaled video image to
	 * be fed to DeSTIN. This pointer should not be deleted.
	 * Points to a float array of length width*height.
	 * Pixel values are between 0 and 1.
	 * 0.0 represents white, 1.0 represents black.
	 *
	 */
	float * getOutput() {
		return float_frame;
	}
	/**
	 * Shows the output of the video or webcam to the screen in a window
	 */
	//see http://opencv.willowgarage.com/documentation/cpp/user_interface.html#cv-namedwindow
	void enableDisplayWindow() {//don't know how to unshow it yet
		cv::namedWindow(DESTIN_VIDEO_WINDOW_TITLE);
	}

	/**
	 * When set to true, "Canny" edge detection is applied to the video source
	 * ( shape outlines are shown in video)
	 */
	void setDoesEdgeDetection(bool on_off){
		edge_detection = on_off;
	}
	/*
	 * grab - grabs a frame from the video source.
	 * Returns true if it could retrieve one, otherwize returns false
	 *
	 * Ment to be used in a while loop to keep capturing until the end of the video.
	 */
	bool grab();
};

#endif /* VIDEOSOURCE_H_ */

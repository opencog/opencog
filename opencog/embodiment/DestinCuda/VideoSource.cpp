/*
 * VideoSource.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: ted
 */

#include "VideoSource.h"
#include <iostream>

using namespace std;


bool VideoSource::grab() {
	if (cap->grab()) {
		cap->retrieve(original_frame); //retrieve the captured frame

		original_size = original_frame.size();

		// see http://opencv.willowgarage.com/documentation/cpp/imgproc_geometric_image_transformations.html#resize
		cv::resize(original_frame, rescaled_frame, target_size ,1.0,1.0); //resize image to target_size

		cvtColor(rescaled_frame, greyscaled_frame, CV_BGR2GRAY); //turn the image grey scale

		//convert the greyscaled_frame into a float array for DeSTIN
		convert(greyscaled_frame, this->float_frame);

		if(edge_detection){
			cv::GaussianBlur(greyscaled_frame, greyscaled_frame, cv::Size(7,7) , .75, .75); // blur the image
			cv::Canny(greyscaled_frame, greyscaled_frame, 0, 30, 3); // apply edge detection
		}

		cv::imshow( DESTIN_VIDEO_WINDOW_TITLE, greyscaled_frame); //show video to output window

		// some strange issues with waitkey, see http://opencv.willowgarage.com/wiki/documentation/c/highgui/WaitKey
		//Needs this so it gives time for the computer to update. If its too small, it wont be drawn at all,
		//if its too high, then the frames per seconds drops.

		if(cv::waitKey(5)>=0){ //stop the video when a key is pressed
			return false;
		}
		return true;
	} else {
	    if(cap->get(CV_CAP_PROP_FRAME_COUNT) > 0){
	        if(cap->get(CV_CAP_PROP_POS_AVI_RATIO) > 0.9){   // if reached near end of video
	            cap->set(CV_CAP_PROP_POS_FRAMES,0);         // move to begining
	            if(cap->grab()){                            // if can get a frame
	                cout << "video rewind " << endl;
	                return this->grab();                    // get the next frame
	            }else{
	                cout << "couldn't rewind and grab first frame." << endl;
	            }
	        }else{
	            cout << "couldn't get video and not end of video. was: " <<  cap->get(CV_CAP_PROP_POS_AVI_RATIO) << endl;
	        }
	    }else{
	        cout << "frame count not larger than 0\n";
	    }

		cout << "Reached end of video stream." << endl;
		return false;
	}
}

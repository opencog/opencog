

#include "VideoSource.h"
#include <iostream>

using namespace std;
int main(int argc, char ** argv){
	
	VideoSource vs(true,"",0);// constructor decides if it shows a video or captures from a webcam 
	vs.enableDisplayWindow();

	while(vs.grab()){
		//uses highgui to show webcam / video output to a window
	}
	return 0;
}

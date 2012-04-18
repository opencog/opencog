#ifndef TRANSPORTER_H_
#define TRANSPORTER_H_

#include <iostream>
#include <stdexcept>
/**
 * This  Transporter class transforms a source image and then transfers it to GPU memory.
 * It allocates its own memory on the device upon creation then frees it in the deconstructor.
 * By default this base class does not transform it, just transers it over.
 * Derived classes may want to arrage the image pixels so that different nodes
 * get different regions of pixels.
 */

#define CUDA_TEST_MALLOC( p, s )                                                                    \
	if( cudaMalloc( p , s ) != 0 ){                                                                 \
		 stringstream mess; mess << "could not cudaMaclloc at " << __FILE__ << ":" << __LINE__ ;    \
	     throw runtime_error(mess.str());}


using namespace std;
class Transporter {
private:
	float * deviceDest;
	int floatArrayLength;
protected:
	float * sourceImage;
	float * transformedImage;

	virtual void transform(){
		transformedImage = sourceImage;
	}

public:
	virtual ~Transporter(){
		cudaFree(deviceDest);
	}

	Transporter(int floatArrayLength )
		:floatArrayLength(floatArrayLength),
		 transformedImage(NULL){

		std::cout << "Transporter constructor" << std::endl;
		cout << "floatArrayLength " << floatArrayLength  << endl;
		CUDA_TEST_MALLOC( (void**)&deviceDest, floatArrayLength  * sizeof(float) );
	}

	void setHostSourceImage(float * sourceImage){
		this->sourceImage = sourceImage;
	}

	float * getDeviceDest(){
		return deviceDest;
	}

	void transport(){
		transform();
		cudaMemcpy( deviceDest, transformedImage, floatArrayLength *sizeof(float), cudaMemcpyHostToDevice );
	}

};

class ImageTransporter : public Transporter {
private:
	const int nodes_wide;
	const int nodes_high;
	const int ppn_x; //pixels per node in the x direction
	const int ppn_y; //pixels per node in the y direction

protected:
	/**
	 * transform - takes the source image, then rearranges it so that
	 * the DeSTIN input nodes will each have a ppnx by ppny image input region
	 */
	virtual void transform(){
		const int image_width = nodes_wide * ppn_x;
		int i = 0;
		int imx; //image x
		int imy; //image y

		for(int ny = 0 ; ny < nodes_high ; ny++ ){
			for(int nx = 0 ; nx < nodes_wide ; nx++ ){
				for(int y  = 0 ; y < ppn_y ; y++ ){
					for(int x = 0 ; x < ppn_x ; x++ ){
						imx = nx * ppn_x + x;
						imy = ny * ppn_y + y;
						transformedImage[i] = sourceImage[image_width * imy + imx];
						i++;
					}
				}
			}
		}//end for ny
	}// end transform
public:

	virtual ~ImageTransporter(){
		delete [] transformedImage;
		cout << "~ImageTransporter" << endl;
	}
	ImageTransporter(int floatArrayLength, int nodes_wide, int nodes_high, int ppn_x, int ppn_y)
		:Transporter(floatArrayLength),
		 nodes_wide(nodes_wide),
		 nodes_high(nodes_high),
		 ppn_x(ppn_x),ppn_y(ppn_y) {
		std::cout << "ImageTransporter constructor" << std::endl;
		if(floatArrayLength != (nodes_wide * ppn_x * nodes_high * ppn_y)){
			std::stringstream mess; mess << __FILE__ ":" << __LINE__ << ": floatArrayLength did not match constructor parameters." << std::endl;
			throw std::logic_error(mess.str());
		}
		transformedImage = new float[floatArrayLength];
	}

};
#endif //TRANSPORTER_H_

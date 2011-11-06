#ifndef InputTransporter_H_
#define InputTransporter_H_

/**
 * This  Transporter class transforms a source image and then transfers it to GPU memory.
 * It allocates its own memory on the device upon creation then frees it in the deconstructor.
 * By default this base class does not transform it, just transers it over.
 * Derived classes may want to arrage the image pixels so that different nodes
 * get different regions of pixels.
 */
class Transporter {
private:
	float * deviceDest;

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

	Transporter(int floatArrayLength ){
		cudaMalloc( (void**)&deviceDest, floatArrayLength  * sizeof(float) );
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
protected:
	virtual void transform(){

		transformedImage = sourceImage;

	}
public:
	void setNodeRegion(int width, height){

	}

};
#endif //InputTransporter_H_

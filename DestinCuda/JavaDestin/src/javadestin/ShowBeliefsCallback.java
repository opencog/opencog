package javadestin;


/**
 * ASCIIArt
 * Not so great attempt to show belief vectors as ASCII art greyscale ribon.
 * 
 */
class ASCIIArt {
	//String blackRamp = "M#E8Oo+i=I;:~.` ";

	static final String blackRamp = "$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,\"^`'. ";
	static final int cellWidth = 4;
	static final float interval = 1.0f / blackRamp.length();

	static float[] toJavaArray(SWIGTYPE_p_float floatPointer, int len) {
		float a[] = new float[len];
		SWIG_FloatArray wfa = SWIG_FloatArray.frompointer(floatPointer);
		for (int i = 0; i < len; i++) {
			a[i] = wfa.getitem(i);
		}
		return a;
	}

	static void printArray(float[] v) {
		for (int k = 0; k < cellWidth; k++) {
			for (int i = 0; i < v.length; i++) {
				int br = (int) (v[i] / interval);
				br = br < blackRamp.length() ? br : br - 1;
				br = blackRamp.length() - br - 1; //reverse the index so it goes from light to dark.
				for (int j = 0; j < cellWidth; j++) {
					System.out.print(blackRamp.charAt(br));
				}
			}
			System.out.println();
		}
	}
}


public class ShowBeliefsCallback extends LayerFinishedCallback {

	FPSCallback fps = new FPSCallback();
	
	@Override
	public void callback(RunningInfo info, DestinKernel layer) {
		//nodeBeliefs
		fps.callback(info, layer); //report fps
		
		if (info.getLayer() != 7 ) {
			return;
		}

		int states = layer.GetNumberOfStates();
		if(info.getSequence_step()==0){
			System.out.println("-----------------------------------------------");
		}
			
		ASCIIArt.printArray(ASCIIArt.toJavaArray(layer.GetNodeBeliefVector(0, 0), states));
	}
}

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
/**
 *
 * @author ted
 */
package javadestin;

import java.text.DecimalFormat;

/**
 * ASCIIArt
 * Used to show belief vectors as ASCII art greyscale ribon.
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

public class JavaDestin {

	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}

	/**
	 * @param args the command line arguments
	 */
	public static void main(String[] args) {
		// TODO code application logic here
		DestinCuda dc = new DestinCuda();
		dc.SetLayerFinishedCallback(new LayerFinishedCallback() {

			DecimalFormat df = new DecimalFormat("0.00");

			@Override
			public void callback(RunningInfo info, DestinKernel layer) {
				//nodeBeliefs
				if (info.getLayer() != 3 || info.getImage_count() < 118) {
					return;
				}

				int states = layer.GetNumberOfStates();
				if(info.getSequence_step()==0){
					System.out.println("-----------------------------------------------");
				}
					
				ASCIIArt.printArray(ASCIIArt.toJavaArray(layer.GetNodeBeliefVector(0, 0), states));
			}
		});

		CommandArgsStuc cas = new CommandArgsStuc();
		cas.setSCodeWord("00010100000");
		cas.setMAX_CNT(120);
		cas.setParametersFileName("/home/ted/oc/destin/src/DestinCuda/config.xml");
		cas.setSeed(Integer.parseInt(cas.getSCodeWord()));
		cas.setStrDestinTrainingFileName("/home/ted/oc/destin/data/MNISTTraining32");
		cas.setStrTesting("/home/ted/oc/destin/data/MNISTTraining32_TESTING");
		cas.setStrDestinNetworkFileToRead("/home/ted/oc/destin/src/DiagnosticData/DestinDiagnostics.xml");
		dc.MainDestinExperiments(cas);



	}
}

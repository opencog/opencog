package callbacks;

import javadestin.*;


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


public class ShowBeliefsCallback extends DestinIterationFinishedCallback {

	private FPSCallback fps = new FPSCallback();
	
	@Override
	public void callback(RunningInfo info, INetwork network) {
		//show frames per second
		fps.callback(info, network); 

		int states = network.getBeliefsPerNode(7);
		if(info.getSequence_step()==0){
			System.out.println("-----------------------------------------------");
		}
			
		ASCIIArt.printArray(Util.toJavaArray(network.getNodeBeliefs(7, 0, 0) , states));
	}
}

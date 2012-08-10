package callbacks;

import java.text.DecimalFormat;
import javadestin.*;


public class BeliefGraphCallback extends DestinIterationFinishedCallback {
	private FPSCallback fps = new FPSCallback(0);
        private DecimalFormat df = new DecimalFormat("0.0000");
	
@Override
	public void callback(INetwork network) {
        int layer = 7;
		int states = network.getBeliefsPerNode(layer);
		float [] beliefs = Util.toJavaArray(network.getNodeBeliefs(layer, 0, 0), states);
		
		int beliefWidth = 80; // how many character columns in chart line, doesn't
                                      // have to match the size of the belief vector
		System.out.print("\033[2J");//clearscreen
		System.out.flush();
		
                System.out.println("states:"+states);
		fps.callback(network);
		
		for(int s = 0 ; s < states; s++){
			int b = (int)(beliefs[s] * beliefWidth);
                        System.out.print(s+":"+df.format(beliefs[s])+":");
			for(int i = 0 ; i < b ; i++){
				System.out.print("X");
			}
			System.out.println();
		}
		System.out.println("-------------------");
		
	}	

}

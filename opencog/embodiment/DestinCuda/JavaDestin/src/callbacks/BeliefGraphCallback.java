package callbacks;

import javadestin.*;


public class BeliefGraphCallback extends DestinIterationFinishedCallback {
	private FPSCallback fps = new FPSCallback(0);
	
@Override
	public void callback(INetwork network) {
                
		int states = network.getBeliefsPerNode(7);
		float [] beliefs = Util.toJavaArray(network.getNodeBeliefs(7, 0, 0), states);
		
		int beliefWidth = 20;
		System.out.print("\033[2J");//clearscreen
		System.out.flush();
		
                System.out.println("states:"+states);
		fps.callback(network);
		
		for(int s = 0 ; s < states; s++){
			int b = (int)(beliefs[s] * beliefWidth);
                        System.out.print(s+":"+beliefs[s]+":");
			for(int i = 0 ; i < b ; i++){
				System.out.print(s);
			}
			System.out.println();
		}
		System.out.println("-------------------");
		
	}	

}

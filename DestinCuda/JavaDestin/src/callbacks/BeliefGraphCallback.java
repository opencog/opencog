package callbacks;

import javadestin.DestinKernel;
import javadestin.LayerFinishedCallback;
import javadestin.RunningInfo;
import javadestin.Util;


public class BeliefGraphCallback extends LayerFinishedCallback {
	private FPSCallback fps = new FPSCallback();
	
@Override
	public void callback(RunningInfo info, DestinKernel layer) {
		if(info.getLayer()!=7) return;
		
		int states = layer.GetNumberOfStates();
		float [] beliefs = Util.toJavaArray(layer.GetNodeBeliefVector(0, 0), states);
		
		int beliefWidth = 20;
		System.out.print("\033[2J");
		System.out.flush();
		
		fps.callback(info, layer);
		
		for(int s = 0 ; s < states; s++){
			int b = (int)(beliefs[s] * beliefWidth);
			for(int i = 0 ; i < b ; i++){
				System.out.print(s);
			}
			System.out.println();
		}
		System.out.println("-------------------");
		
	}	

}

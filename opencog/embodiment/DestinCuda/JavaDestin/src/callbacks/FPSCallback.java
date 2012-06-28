package callbacks;

import javadestin.DestinIterationFinishedCallback;
import javadestin.INetwork;
import javadestin.RunningInfo;

public class FPSCallback extends DestinIterationFinishedCallback{
	long lastTime =  System.currentTimeMillis();
	long lastReportTime = lastTime;
	
	@Override
	public void callback(RunningInfo info, INetwork network) {
		
		long time = System.currentTimeMillis();
		long diff = time - lastReportTime; 
		//if(diff > 1000){
			System.out.println("FPS: "+ 1000.0 / (time - lastTime));
		//	lastReportTime = time;
		//}
		lastTime = time;
		  
	}
}
package callbacks;

import javadestin.DestinKernel;
import javadestin.LayerFinishedCallback;
import javadestin.RunningInfo;

public class FPSCallback extends LayerFinishedCallback{
	long lastTime =  System.currentTimeMillis();
	long lastReportTime = lastTime;
	
	@Override
	public void callback(RunningInfo info, DestinKernel layer) {
		if(info.getLayer()!=7) {
			return;
		}
		
		long time = System.currentTimeMillis();
		long diff = time - lastReportTime; 
		//if(diff > 1000){
			System.out.println("FPS: "+ 1000.0 / (time - lastTime));
		//	lastReportTime = time;
		//}
		lastTime = time;
		  
	}
}
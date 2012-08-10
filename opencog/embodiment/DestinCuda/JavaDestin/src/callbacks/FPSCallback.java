package callbacks;

import javadestin.DestinIterationFinishedCallback;
import javadestin.INetwork;

public class FPSCallback extends DestinIterationFinishedCallback{
	long lastReportTime = System.currentTimeMillis();
        long frameCount = 0;
	long delay = 0;
        
        public FPSCallback(long delay){
            this.delay = delay;
        }
        
	@Override
	public void callback(INetwork network) {
		long time = System.currentTimeMillis();
		if(time - lastReportTime > delay){
                    System.out.println("FPS: "+ 1000*(float)frameCount / (float) (time - lastReportTime));
                    lastReportTime = time;
                    frameCount = 0;
		}
                frameCount++;
	}
}
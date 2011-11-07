package javadestin;

import javadestin.NetworkCreator.Network;
import javadestin.NetworkCreator.NetworkConfig;


class FPSCallback extends LayerFinishedCallback{
	long lastTime =  System.currentTimeMillis();
	long lastReportTime = lastTime;
	
	@Override
	public void callback(RunningInfo info, DestinKernel layer) {
		if(info.getLayer()!=0) {
			return;
		}
		
		long time = System.currentTimeMillis();
		long diff = time - lastReportTime; 
		if(diff > 1000){
			System.out.println("FPS: "+ 1000.0 / (time - lastTime));
			lastReportTime = time;
		}
		lastTime = time;
		  
	}
}
public class VideoExperiment {
	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}
	
	static void run(){

		NetworkCreator nc = new NetworkCreator();
		
		NetworkConfig config = new NetworkConfig();
		
		Network n = nc.createStandard(new int[]{20,16,14,12,10,8,6,4}, 16, config); //4,6,8,10,12,14,16,20
		
		n.setLayerFinishedCallback( new ShowBeliefsCallback() );
		
		VidSource s = new VidSource(true,"");
		s.enableDisplayWindow();//shows the video to the screen while feeding to DeSTIN
		
		
		VideoPresentor vp = new VideoPresentor();
		vp.setNetwork(n);
		vp.setSource(s);
		
		//currently destin only supports square layers
		int layerNodesWide = 128;
		int layerNodesHigh = 128;
		int nodePixelWidth = 4;
		int nodePixelHeight = 4;
		
		int pixelCount =  layerNodesWide  * layerNodesHigh * nodePixelWidth * nodePixelHeight;
		
		vp.setInputTransporter(new ImageTransporter(pixelCount, layerNodesWide ,layerNodesHigh , nodePixelWidth , nodePixelHeight));
		
		vp.present(); //present the video source to DeSTIN for processing	
	}
	
	public static void main(String [] argv) throws InterruptedException{
		try{
			run();
		}catch(Exception e){
			System.gc();//let the c++ destructors be called... not sure that graphics card memory would be cleared properly when jvm exits immediatly.
			e.printStackTrace();
			Thread.sleep(1000);
		}
	}
}

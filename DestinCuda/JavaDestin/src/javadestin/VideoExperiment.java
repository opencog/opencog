package javadestin;

import javadestin.NetworkCreator.Network;
import javadestin.NetworkCreator.NetworkConfig;

public class VideoExperiment {

	public static void main(String [] argv){
	
		NetworkCreator nc = new NetworkCreator();
		
		NetworkConfig config = new NetworkConfig();
		
		Network n = nc.createStandard(new int[]{20,16,14,12,10,8,6,4}, 16, config); //4,6,8,10,12,14,16,20
		
		n.setLayerFinishedCallback(new LayerFinishedCallback(){
			long lastTime =  System.currentTimeMillis();
			@Override
			public void callback(RunningInfo info, DestinKernel layer) {
				long time = System.currentTimeMillis();
				
				System.out.println("FPS: "+ 1000.0 / (time - lastTime));
				lastTime = time;
			}
		});
		
		VidSource s = new VidSource(true,"",1);
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
}

package javadestin;

import javadestin.NetworkCreator.Network;

public class VideoExperiment {

	public static void main(String [] argv){
	
		NetworkCreator nc = new NetworkCreator();
		Network n = nc.createStandard(new int[]{25,15,15,10});
		
		VideoPresentor vp = new VideoPresentor();
		vp.setNetwork(n);
		
		VidSource s = new VidSource(true,"",1);
		s.showVideo(); //shows the video to the screen while feeding to DeSTIN
		
		vp.setSource(s);
		
		vp.setLayerFinishedCallback(new ShowBeliefsCallback() );
		
		vp.present(); //present the video source to DeSTIN for processing
	}
}

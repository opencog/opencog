package javadestin;
import javadestin.NetworkCreator.Network;

public class VideoPresentor implements Presentor {
	Source source;
	Network network;
	LayerFinishedCallback lfcb;
	
	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}
	
	
	@Override
	public void present(){
		if(network==null){
			throw new RuntimeException("Set the network to present to first.");
		}
		
		if(source==null){
			throw new RuntimeException("Video source needs to be set.");
		}
		
		while(source.grab()){
			System.out.println("grabed frame"); 
		}
		System.out.println("finished presenting");
	}
	

	@Override
	public void setNetwork(Network n) {
		this.network = n;
	}

	@Override
	public void setSource(Source s) {
		// TODO Auto-generated method stub
		this.source = s;
	}

	@Override
	public void setLayerFinishedCallback(LayerFinishedCallback lfc) {
		this.lfcb = lfc;
	}
}

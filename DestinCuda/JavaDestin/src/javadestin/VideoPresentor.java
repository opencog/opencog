package javadestin;
import javadestin.NetworkCreator.Network;

public class VideoPresentor implements Presentor {
	Source source;
	Network network;
	LayerFinishedCallback lfcb;
	Transporter  inputTrans;
	
	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}
	
	
	@Override
	public void present(){
		if(network==null){
			throw new RuntimeException("Network cannot be null.");
		}
		if(source==null){
			throw new RuntimeException("Video cannot be null.");
		}
		if(inputTrans==null){
			throw new RuntimeException("Transporter cannot be null."); 
		}
		
		while(source.grab()){
			
			inputTrans.setHostSourceImage(source.getOutput());
			
			//arrange pixel groups to match input node regions
			//and copy the array to the device
			inputTrans.transport();
			network.doDestin(inputTrans.getDeviceDest());
			
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
	public void setInputTransporter(Transporter t) {
		inputTrans = t;
	}
}

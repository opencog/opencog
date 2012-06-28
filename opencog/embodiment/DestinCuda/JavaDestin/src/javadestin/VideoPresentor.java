package javadestin;

public class VideoPresentor implements IPresentor {
	private ISource source;
	private INetwork network;
	private Transporter  inputTrans;
	
	private boolean isStopped;
	final private Object networkLock = new Object();
	
	@Override
	public Object getNetworkLock(){
		return networkLock;
	}
	
	@Override
	public void present(){
		isStopped = false;
		if(network==null){
			throw new RuntimeException("Network cannot be null.");
		}
		if(source==null){
			throw new RuntimeException("Video cannot be null.");
		}
		if(inputTrans==null){
			throw new RuntimeException("Transporter cannot be null."); 
		}
		
		while(!isStopped && source.grab()){
			
			inputTrans.setHostSourceImage(source.getOutput());
			
			//arrange pixel groups to match input node regions
			//and copy the array to the device
			inputTrans.transport();

			//Make sure no other threads are trying to do stuff with the network
			synchronized (networkLock) {
				network.doDestin(inputTrans.getDeviceDest());
			}
		}
		System.out.println("finished presenting");
	}
	

	@Override
	public void setNetwork(INetwork n) {
		this.network = n;
	}

	@Override
	public void setSource(ISource s) {
		this.source = s;
	}


	@Override
	public void setInputTransporter(Transporter t) {
		inputTrans = t;
	}

	@Override
	public void stop() {
		isStopped = true;
	}
}

package javadestin;

import javadestin.Network.NetworkConfig;
import callbacks.BeliefGraphCallback;



public class VideoExperiment {
	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}
	
	private Presentor presentor;
	private Network network;
	private Thread presentingThread;
	
	public VideoExperiment(){
		VidSource s = new VidSource(true,"");
		s.enableDisplayWindow();//shows the video to the screen while feeding to DeSTIN
		
		presentor = new VideoPresentor();
		network = newNetwork();
		presentor.setNetwork(network);
		presentor.setSource(s);
		
		//currently destin only supports square layers
		int layerNodesWide = 128;
		int layerNodesHigh = 128;
		int nodePixelWidth = 4;
		int nodePixelHeight = 4;
		
		int pixelCount =  layerNodesWide  * layerNodesHigh * nodePixelWidth * nodePixelHeight;
		
		presentor.setInputTransporter(new ImageTransporter(pixelCount, layerNodesWide ,layerNodesHigh , nodePixelWidth , nodePixelHeight));
	}
	
	private Network newNetwork(){
		NetworkConfig config = new NetworkConfig();
		Network network = new Network(new int[]{20,16,14,12,10,8,6,2}, 16, config); //4,6,8,10,12,14,16,20
		network.setLayerFinishedCallback( new BeliefGraphCallback() );
		return network; 
	}
	/**
	 * reset - the network looses all its training 
	 * and centroids are randomized and PSSA tables 
	 * are reset to uniform;
	 */
	public void reset(){
		synchronized (presentor.getNetworkLock()) {
			network.free();
			network = newNetwork();
			presentor.setNetwork(network);
		}
	}
	
	/**
	 * The video stream stops and network stops recieving input
	 */
	public void stop(){
		presentor.stop();
	}
	/**
	 * The video stream starts and is fed to the presentor
	 */
	public void start(){
		if(presentingThread!=null && presentingThread.isAlive() ){
			throw new IllegalStateException("The source is already being presented to the network");
		}
		
		presentingThread = new Thread(new Runnable() {
			@Override
			public void run() {
				presentor.present(); //present the video source to DeSTIN for processing	
				// TODO Auto-generated method stub
				
			}
		});
		presentingThread.start();
	}
	
	public void setPOSTraining(boolean training){
		network.setIsPOSTraining(training);
	}
	public void setPSSATraining(boolean training){
		network.setIsPSSATraining(training);
	}
	public static void main(String [] argv) throws InterruptedException{
		try{
			new VideoExperiment().start();
		}catch(Exception e){
			e.printStackTrace();
		}
		System.gc();//let the c++ destructors be called... not sure that graphics card memory would be cleared properly when jvm exits immediatly.
		Thread.sleep(1000);
		System.out.print("Main exiting");
	}
}

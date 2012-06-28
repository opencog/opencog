package javadestin;



public class VideoExperiment implements IExperiment {
	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}
	
	private IPresentor presentor;
	private INetwork network;
	private Thread presentingThread;
	
        private INetworkFactory networkFactory;
        
	public VideoExperiment(INetworkFactory factory){
                networkFactory = factory;
		VidSource s = new VidSource(true,"");
		s.enableDisplayWindow();//shows the video to the screen while feeding to DeSTIN
		
		presentor = new VideoPresentor();
                
                network = networkFactory.create();
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
	

	/**
	 * reset - the network looses all its training 
	 * and centroids are randomized and PSSA tables 
	 * are reset to uniform;
	 */
    @Override
	public void reset(){
		synchronized (presentor.getNetworkLock()) {
			network.free();
			network = networkFactory.create();
			presentor.setNetwork(network);
		}
	}
	
	/**
	 * The video stream stops and network stops recieving input
	 */
    @Override
	public void stop(){
		presentor.stop();
	}
	/**
	 * The video stream starts and is fed to the presentor
	 */
    @Override
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
	
    @Override
	public void setPOSTraining(boolean training){
		network.setIsPOSTraining(training);
	}
    @Override
	public void setPSSATraining(boolean training){
		network.setIsPSSATraining(training);
	}
    public static void main(String[] argv) throws InterruptedException {
        try {
            new VideoExperiment(new NetworkFactory(true)).start();
        } catch (Exception e) {
            e.printStackTrace();
        }
        System.gc();//let the c++ destructors be called... not sure that graphics card memory would be cleared properly when jvm exits immediatly.
	Thread.sleep(1000);
	System.out.print("Main exiting");
    }
}

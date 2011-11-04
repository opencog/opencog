package javadestin;

public class VideoPresentor {
	Network network;
	VideoSource source;
	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}
	
	public void setNetwork(Network network){
		this.network = network;
	}
	
	public void setVideoSource(VideoSource source){
		this.source = source;
	}
	public void isOpened(){
		
	}
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
	
	public static void main(String [] argv){
		VideoPresentor vp = new VideoPresentor();
		VideoSource vs = new VideoSource(true, "",1);
		vp.setVideoSource(vs);
		vp.setNetwork(new Network());
		if(vs.isOpened()){
			vs.showVideo();
			vp.present();
		}else{
			System.out.println("Could not open the video source.");
		}
			
		
	}
}

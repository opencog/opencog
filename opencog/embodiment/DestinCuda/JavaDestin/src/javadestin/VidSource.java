package javadestin;

public class VidSource extends VideoSource implements ISource {

	public VidSource(boolean use_device, String video_file) {
		super(use_device, video_file);
	}
	
	public VidSource(boolean use_device, String video_file, int dev_no){
		super(use_device, video_file, dev_no);
	}
}

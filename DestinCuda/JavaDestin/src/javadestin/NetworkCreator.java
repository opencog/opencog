package javadestin;

import java.util.List;

public class NetworkCreator {

	public static class Network {
		LayerFinishedCallback callback;
		List<DestinKernel> layers;
		RunningInfo ri = new RunningInfo();
		
		//how many times doDestin is called
		int callcount = 0;
		
		public RunningInfo getRunningInfo(){
			return ri;
		}
		public void doDestin(SWIGTYPE_p_float dInput){
			DestinKernel layer;
			ri.setImage_count(callcount);
			for(int l = layers.size() - 1 ; l >=0 ; l--){
				ri.setLayer(l);
				layer = layers.get(l);
				
				if(l==0){
					layer.DoDestin(dInput, null);
				}else{
					layer.DoDestin( layers.get(l - 1 ).GetDevicePointerBeliefs() , null);
				}
				if(callback!=null){ 
					this.callback.callback(ri, layer); 
				}
			}
			callcount++;
		}
		
		public void setLayerFinishedCallback(LayerFinishedCallback callback){
			this.callback = callback;
		}
		
	}
	
	Network createStandard(int [] centroidCounts){
		return null;
	}
}

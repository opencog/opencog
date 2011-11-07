package javadestin;

import java.util.ArrayList;
import java.util.List;

public class NetworkCreator {

	public static class NetworkConfig{
		//TODO: is this a good default learning rate?
		public float fixedLearningRate = .001f;
	}
	public static class Network {
		private LayerFinishedCallback callback;
		private List<DestinKernel> layers = new ArrayList<DestinKernel>();
		private RunningInfo ri = new RunningInfo();
		//how many times doDestin is called
		private int callcount = 0;
		
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
	
	Network createStandard(int [] centroidCounts, int pixelsPerInputNode, NetworkConfig config){
		int nlayers = centroidCounts.length;
		
		DestinKernel layer;
		CurandGeneratorWrapper gen = new CurandGeneratorWrapper();
		Network network = new Network();
		
		for(int l = 0 ; l < nlayers ; l++){
			layer = new DestinKernel();
			//TODO: is ID set right?
			int width = (int)Math.pow(2, nlayers - l);
			int inputDim = l==0  ? pixelsPerInputNode : centroidCounts[l-1] * 4;
			int parentStates = l==nlayers - 1 ? 1 : centroidCounts[l+1];
			layer.Create(l, width, width, centroidCounts[l], parentStates, inputDim, config.fixedLearningRate, gen.getReference());
			
			if(l>0){
				network.layers.get(l - 1).SetInputAdvice(layer.GetOutputAdvice());
			}
			if(l==(nlayers - 1)){
				layer.SetInputAdvice(null);
			}
		}
		return network;
	}
}

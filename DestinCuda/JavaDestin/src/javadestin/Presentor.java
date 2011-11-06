package javadestin;

import javadestin.NetworkCreator.Network;

public interface Presentor {
	public void present();
	
	public void setNetwork(Network n);
	
	public void setSource(Source s);
	
	public void setLayerFinishedCallback(LayerFinishedCallback lfc);
	
}

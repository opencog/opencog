/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
/**
 *
 * @author ted
 */
package javadestin;

import callbacks.ShowBeliefsCallback;



public class JavaDestin {

	static {
		//load swig generated destin cuda c++ dynamic library 
		System.loadLibrary("destinjava");
	}

	/**
	 * @param args the command line arguments
	 */
	public static void main(String[] args) {
		// TODO code application logic here
		DestinCuda dc = new DestinCuda();
		dc.SetLayerFinishedCallback(new ShowBeliefsCallback() );
		CommandArgsStuc cas = new CommandArgsStuc();
		cas.setSCodeWord("00010100000");
		cas.setMAX_CNT(120);
		cas.setParametersFileName("/home/ted/oc/destin/src/DestinCuda/config.xml");
		cas.setSeed(Integer.parseInt(cas.getSCodeWord()));
		cas.setStrDestinTrainingFileName("/home/ted/oc/destin/data/MNISTTraining32");
		cas.setStrTesting("/home/ted/oc/destin/data/MNISTTraining32_TESTING");
		cas.setStrDestinNetworkFileToRead("/home/ted/oc/destin/src/DiagnosticData/DestinDiagnostics.xml");
		dc.MainDestinExperiments(cas);



	}
}

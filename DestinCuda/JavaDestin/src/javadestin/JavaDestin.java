/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author ted
 */
package javadestin;


public class JavaDestin {

    static {
        //load swig generated destin cuda c++ dynamic library 
       // System.loadLibrary("cudadestin");
        System.loadLibrary("destinjava");
    }
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        // TODO code application logic here
        DestinCuda dc = new DestinCuda();
        dc.SetLayerFinishedCallback( new LayerFinishedCallback(){
            @Override
            public void callback(int run) {
                //super.callback(run);
                System.out.println("java callback called.");
            }
        
        });
        
        CommandArgsStuc cas = new CommandArgsStuc();
        cas.setSCodeWord("00010100000");
        cas.setMAX_CNT(120);
        cas.setParametersFileName(null);
        dc.MainDestinExperiments(cas);
        
        
    }
}

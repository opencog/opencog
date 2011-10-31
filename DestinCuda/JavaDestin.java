package javadestin;


public class JavaDestin {

    static {
        //load swig generated destin cuda c++ dynamic library 
        System.loadLibrary("cudadestin");
        System.loadLibrary("destinjava");
    }
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        // TODO code application logic here
        DestinKernel dk = new DestinKernel();
       
    }
}

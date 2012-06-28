package javadestin;

public class Util {
        public static void here(){
            StackTraceElement e = Thread.currentThread().getStackTrace()[2];
            System.out.println("I'm here: "+e.getFileName() +":"+ e.getLineNumber());
        }
	public static float[] toJavaArray(SWIGTYPE_p_float floatPointer, int len) {
		float a[] = new float[len];
		SWIG_FloatArray wfa = SWIG_FloatArray.frompointer(floatPointer);
		for (int i = 0; i < len; i++) {
			a[i] = wfa.getitem(i);
		}
		return a;
	}
}

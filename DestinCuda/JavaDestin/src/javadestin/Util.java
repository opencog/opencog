package javadestin;

public class Util {
	public static float[] toJavaArray(SWIGTYPE_p_float floatPointer, int len) {
		float a[] = new float[len];
		SWIG_FloatArray wfa = SWIG_FloatArray.frompointer(floatPointer);
		for (int i = 0; i < len; i++) {
			a[i] = wfa.getitem(i);
		}
		return a;
	}
}

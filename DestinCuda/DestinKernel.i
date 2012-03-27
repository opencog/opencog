
%module(directors="1") DestModule 
%{
#include "LayerFinishedCallback.h"
#include "DestinKernel.h"
#include "DestinCuda.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VideoSource.h"
#include "CurandGeneratorWrapper.h"
#include "Transporter.h"
%}

/* 
turn on director wrapping callback.
See http://www.swig.org/Doc2.0/SWIGDocumentation.html#Java_directors
See https://swig.svn.sourceforge.net/svnroot/swig/trunk/Examples/java/callback/
*/

%include "carrays.i" 
%array_class(int, SWIG_IntArray);
%array_class(float, SWIG_FloatArray);

%include "std_string.i"

%feature("director") LayerFinishedCallback;

%include "VideoSource.h"
%include "LayerFinishedCallback.h"
%include "DestinKernel.h"
%include "DestinCuda.h"
%include "CurandGeneratorWrapper.h"
%include "Transporter.h"

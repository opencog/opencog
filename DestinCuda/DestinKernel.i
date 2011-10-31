
%module(directors="1") DestModule 
%{
#include "LayerFinishedCallback.h"
#include "DestinKernel.h"
#include "DestinCuda.h"
%}

/* 
turn on director wrapping callback.
See http://www.swig.org/Doc2.0/SWIGDocumentation.html#Java_directors
See https://swig.svn.sourceforge.net/svnroot/swig/trunk/Examples/java/callback/
*/

%include "std_string.i"

%feature("director") LayerFinishedCallback;


%include "LayerFinishedCallback.h"
%include "DestinKernel.h"

/* %include "StringArrayToCharPP.i" 
*/
%include "DestinCuda.h"

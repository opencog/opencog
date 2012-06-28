#ifndef __MACROS
#define __MACROS

#define THREADS_MAX 512

// max an min macros
#define min(a,b) a < b ? a : b;
#define max(a,b) a > b ? a : b;

// unsigned int macro
#define uint unsigned int

// a few macros to make cluster_lib.c a bit more portable between C and MEX stuff
/*
#include "mex.h"

#define PRINTF mexPrintf

#define oops(s) { mexErrMsgTxt(s); }
#define MALLOC(s,t,n) {                                 \
    if((s = (t *) mxMalloc(n*sizeof(t))) == NULL) {     \
        oops("error: malloc() ");                       \
    }                                                   \
}                                                       \

#define FREE(t) {                                       \
    if( t != NULL ) {                                   \
        mxFree(t);                                      \
    } else {                                            \
        mexErrMsgTxt("ALREADY NULL");                   \
    }                                                   \
}                                                       \
*/

#define PRINTF printf

#define oops(s) { fprintf(stderr, s); exit(1); }

#define MALLOC(s,t,n) {                                 \
    if((s = (t *) malloc(n*sizeof(t))) == NULL) {       \
        oops("error: malloc()\n");                      \
    }                                                   \
}

#define FREE(t) {                                       \
    if( t != NULL ) {                                   \
        free(t);                                        \
    } else {                                            \
        oops("error: free()\n");                        \
    }                                                   \
}

#define CUDAMALLOC(t,s) {                               \
    cudaMalloc(t,s);                                    \
    CUDACHECKERROR();                                   \
}

#define CUDAFREE(t) {                                   \
    cudaFree(t);                                        \
    CUDACHECKERROR();                                   \
}

#define CUDAMEMCPY(dest,src,size,type) {                \
    cudaMemcpy(dest,src,size,type);                     \
    CUDACHECKERROR();                                   \
}

#define CUDAMEMSET(dest,val,size) {                     \
    cudaMemset(dest, val, size);                        \
    CUDACHECKERROR();                                   \
}


#define CUDACHECKERROR() {                              \
    cudaError_t err = cudaGetLastError();               \
    if( err != cudaSuccess ) {                          \
        PRINTF("Line %d, in %s\n", __LINE__, __FILE__); \
        PRINTF("%s\n", cudaGetErrorString(err));        \
        oops("Cuda API Error");                         \
    }                                                   \
}

#endif

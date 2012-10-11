#ifndef __MACROS
#define __MACROS

// unsigned int macro
#include <sys/types.h>

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

#endif

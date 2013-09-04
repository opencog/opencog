
#ifdef HAVE_CYTHON

// XXX Cython currently conflicts with standard C library defintions.
// The push/pop below should hush it, for now. (needed for cython
// 0.15.1 and maybe other versions)  FIXME someday...
#ifdef _GNU_SOURCE
#pragma push_macro("_POSIX_C_SOURCE")
#pragma push_macro("_XOPEN_SOURCE")
#undef _POSIX_C_SOURCE
#undef _XOPEN_SOURCE
#endif
#include <Python.h>
#ifdef _GNU_SOURCE
#pragma pop_macro("_POSIX_C_SOURCE")
#pragma pop_macro("_XOPEN_SOURCE")
#endif

#endif /* HAVE_CYTHON */

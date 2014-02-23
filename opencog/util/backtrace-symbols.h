
#ifndef _BACKTRACE_SYMBOLS_H_
#define _BACKTRACE_SYMBOLS_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAVE_BFD
char **oc_backtrace_symbols(void *const *buffer, int size);
void oc_backtrace_symbols_fd(void *const *buffer, int size, int fd);
#else
char **oc_backtrace_symbols(void *const *buffer, int size) { return NULL; }
void oc_backtrace_symbols_fd(void *const *buffer, int size, int fd) {}
#endif

#ifdef __cplusplus
};
#endif

#endif // _BACKTRACE_SYMBOLS_H_

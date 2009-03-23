#ifndef _VSSCANF_H_
#define _VSSCANF_H_

#include <stdarg.h>

//int vsscanf( const char *str, const char *format, va_list arglist);
int vsscanf( char *buf, char *format, va_list argp );
static char *Advance( char *bufp );

#endif // #ifndef _VSSCANF_H_

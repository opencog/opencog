#ifndef _WCWIDTH_H
#define _WCWIDTH_H

#include <wchar.h>

int mk_wcwidth(wchar_t);
int mk_wcswidth(const wchar_t *, size_t);
#endif /* _WCWIDTH_H */

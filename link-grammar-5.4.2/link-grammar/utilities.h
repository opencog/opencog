/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2009-2013 Linas Vepstas                                 */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/
#ifndef _LINK_GRAMMAR_UTILITIES_H_
#define _LINK_GRAMMAR_UTILITIES_H_

/* The _Win32 definitions are for native-Windows compilers.  This includes
 * MSVC (only versions >=14 are supported) and MINGW (under MSYS or Cygwin).
 * The _WIN32 definitions are not for Cygwin, which doesn't define _WIN32. */

#include <ctype.h>
#include <stdio.h>
#ifdef _WIN32
#define _CRT_RAND_S
#endif /* _WIN32 */
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <wctype.h>
#include <locale.h>
#ifdef HAVE_LOCALE_T_IN_XLOCALE_H
#include <xlocale.h>
#endif /* HAVE_LOCALE_T_IN_XLOCALE_H */

#include "error.h"
#include "lg_assert.h"

#ifdef HAVE_ALLOCA_H
# include <alloca.h>
#elif defined __GNUC__
#ifndef alloca
# define alloca __builtin_alloca
#endif /* !alloca */
#elif defined _AIX
# define alloca __alloca
#elif defined _MSC_VER
# include <malloc.h>
# define alloca _alloca
#else
# include <stddef.h>
# ifdef  __cplusplus
extern "C"
# endif
void *alloca (size_t);
#endif

#ifndef TLS
#ifdef _MSC_VER
#define TLS __declspec(thread)
#else
#define TLS
#endif /* _MSC_VER */
#endif /* !TLS */

#ifndef strdupa
/* In the following, the argument should not have side effects. */
#define strdupa(s) strcpy(alloca(strlen(s)+1), s)
#endif

/* Windows, POSIX and GNU have different ideas about thread-safe strerror(). */
#ifdef _WIN32
#define strerror_r(errno, buf, len) strerror_s(buf, len, errno)
#else
#ifdef _GNU_SOURCE
/* Emulate the POSIX version; assuming len>0 and a successful call. */
#define strerror_r(errno, buf, len) abs((strcpy(buf, strerror_r (errno, buf, len)), 0))
#endif /* _GNU_SOURCE */
#endif /* _WIN32 */

#ifdef _MSC_VER
/* These definitions are incorrect, as these functions are different(!)
 * (non-standard functionality).
 * See http://stackoverflow.com/questions/27754492 . Fortunately,
 * MSVC 14 supports C99 functions so these definitions are now unneeded.
 * (LG library compilation is unsupported for previous MSVC versions.)
 * (Left here for documentation.) */
#if 0
/* MS Visual C uses non-standard string function names */
#define snprintf _snprintf
#define vsnprintf _vsnprintf
#endif

/* Avoid plenty of: warning C4090: 'function': different 'const' qualifiers.
 * This happens, for example, when the argument is "const void **". */
#define free(x) free((void *)x)
#define realloc(x, s) realloc((void *)x, s)
#define memcpy(x, y, s) memcpy((void *)x, (void *)y, s)
#define qsort(x, y, z, w) qsort((void *)x, y, z, w)
#endif /* _MSC_VER */

#if defined(HAVE_LOCALE_T_IN_LOCALE_H) || defined(HAVE_LOCALE_T_IN_XLOCALE_H)
#define HAVE_LOCALE_T 1
#endif /* HAVE_LOCALE_T_IN_LOCALE_H || HAVE_LOCALE_T_IN_XLOCALE_H) */

#ifdef _WIN32
#include <windows.h>
#include <mbctype.h>

/* Compatibility definitions. */
#ifndef strncasecmp
#define strncasecmp(a,b,s) strnicmp((a),(b),(s))
#endif
/* Note that "#define _CRT_RAND_S" is needed before "#include <stdlib.h>" */
#define rand_r(seedp) rand_s(seedp)

/* No strtok_s in these versions and their strtok_r is incompatible. */
#if _WINVER != 0x501 /* XP */ && _WINVER != 0x502 /* Server 2003 */
#define strtok_r strtok_s
#define HAVE_STRTOK_R
#endif

/* Native windows has locale_t, and hence HAVE_LOCALE_T is defined here.
 * However, MinGW currently doesn't have locale_t. If/when it has locale_t,
 * "configure" will define HAVE_LOCALE_T for it. */
#ifndef __MINGW32__
#define HAVE_LOCALE_T
#endif

#ifdef HAVE_LOCALE_T
#define locale_t _locale_t
#define iswupper_l  _iswupper_l
#define iswalpha_l  _iswalpha_l
#define iswdigit_l  _iswdigit_l
#define iswspace_l  _iswspace_l
#define towlower_l  _towlower_l
#define towupper_l  _towupper_l
#define freelocale _free_locale
#endif /* HAVE_LOCALE_T */

/* strndup() is missing in Windows. */
char * strndup (const char *str, size_t size);

/* Users report that the default mbrtowc that comes with windows and/or
 * cygwin just doesn't work very well. So we use our own custom version,
 * instead.
 */
#ifdef mbrtowc
#undef mbrtowc
#endif
size_t lg_mbrtowc(wchar_t *, const char *, size_t n, mbstate_t *ps);
#define mbrtowc(w,s,n,x) lg_mbrtowc(w,s,n,x)
#endif /* _WIN32 */

/* MSVC isspace asserts in debug mode, and mingw sometime returns true,
 * when passed utf8. Thus, limit to 7 bits for windows. */
#ifdef _WIN32
  #define lg_isspace(c) ((0 < c) && (c < 127) && isspace(c))
#else
  #define lg_isspace isspace
#endif

#if __APPLE__
/* It appears that fgetc on Mac OS 10.11.3 "El Capitan" has a weird
 * or broken version of fgetc() that flubs reads of utf8 chars when
 * the locale is not set to "C" -- in particular, it fails for the
 * en_US.utf8 locale; see bug report #293
 * https://github.com/opencog/link-grammar/issues/293
 */
static inline int lg_fgetc(FILE *stream)
{
	char c[4];  /* general overflow paranoia */
	size_t nr = fread(c, 1, 1, stream);
	if (0 == nr) return EOF;
	return (int) c[0];
}

static inline int lg_ungetc(int c, FILE *stream)
{
	/* This should work, because we never unget past the newline char. */
	int rc = fseek(stream, -1, SEEK_CUR);
	if (rc) return EOF;
	return c;
}

#else
#define lg_fgetc   fgetc
#define lg_ungetc  ungetc
#endif


#if defined(__sun__)
int strncasecmp(const char *s1, const char *s2, size_t n);
/* This does not appear to be in string.h header file in sunos
   (Or in linux when I compile with -ansi) */
#endif

/* Cygwin < 2.6.0 doesn't have locale_t. */
#ifdef HAVE_LOCALE_T
locale_t newlocale_LC_CTYPE(const char *);
#else
typedef int locale_t;
#define iswupper_l(c, l) iswupper(c)
#define iswalpha_l(c, l) iswalpha(c)
#define iswdigit_l(c, l) iswdigit(c)
#define iswspace_l(c, l) iswspace(c)
#define towlower_l(c, l) towlower(c)
#define towupper_l(c, l) towupper(c)
#define freelocale(l)
#endif /* HAVE_LOCALE_T */

#define STR(x) #x
#define STRINGIFY(x) STR(x)

#if defined(__UCLIBC__)
#define fmaxf(a,b) ((a) > (b) ? (a) : (b))
#endif

#if !defined(MIN)
#define MIN(X,Y)  ( ((X) < (Y)) ? (X) : (Y))
#endif
#if !defined(MAX)
#define MAX(X,Y)  ( ((X) > (Y)) ? (X) : (Y))
#endif

/* From ccan array_size.h and build_assert.h, which are under a CC0 license */
#define BUILD_ASSERT_OR_ZERO(cond) (sizeof(char [1 - 2*!(cond)]) - 1)
#if !defined(ARRAY_SIZE)
/**
 * ARRAY_SIZE - get the number of elements in a visible array
 * @arr: the array whose size you want.
 *
 * This does not work on pointers, or arrays declared as [], or
 * function parameters.  With correct compiler support, such usage
 * will cause a build error (see build_assert).
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]) + _array_size_chk(arr))

#if HAVE_BUILTIN_TYPES_COMPATIBLE_P && HAVE_TYPEOF
/* Two gcc extensions.
 * &a[0] degrades to a pointer: a different type from an array */
#define _array_size_chk(arr)
	BUILD_ASSERT_OR_ZERO(!__builtin_types_compatible_p(typeof(arr), \
							typeof(&(arr)[0])))
#else
#define _array_size_chk(arr) 0
#endif
#endif /* !defined(ARRAY_SIZE) */

/* Optimizations etc. that only gcc understands */
#if __GNUC__ > 2
#define GNUC_MALLOC __attribute__ ((malloc))
#define GNUC_UNUSED __attribute__ ((unused))
#else
#define GNUC_MALLOC
#define GNUC_UNUSED
#endif

/**
 * Return the length, in codepoints/glyphs, of the utf8-encoded
 * string.  The string is assumed to be null-terminated.
 * This is needed when splitting words into morphemes.
 */
static inline size_t utf8_strlen(const char *s)
{
	mbstate_t mbss;
	memset(&mbss, 0, sizeof(mbss));
#if defined(_MSC_VER) || defined(__MINGW32__)
	return MultiByteToWideChar(CP_UTF8, 0, s, -1, NULL, 0)-1;
#else
	return mbsrtowcs(NULL, &s, 0, &mbss);
#endif
}

/**
 * Return the distance, in bytes, to the next character, in the
 * input utf8-encoded string.
 */
static inline size_t utf8_next(const char *s)
{
#ifdef _WIN32
	/* mbrlen does not work correctly on Windows. See issue #285 */
	/* https://github.com/opencog/link-grammar/issues/285 */
	size_t len = 0;
	while (0 != *s)
	{
		if ((0x80 <= ((unsigned char) *s)) &&
		(((unsigned char) *s) < 0xc0)) { s++; len++; }
		else return len+1;
	}
	return len;
#else
	size_t len;
	mbstate_t mbs;
	memset(&mbs, 0, sizeof(mbs));
	len = mbrlen(s, MB_CUR_MAX, &mbs);
	if (len == (size_t)(-1) || len == (size_t)(-2)) {
		/* Too long or malformed sequence, step one byte. */
		return 1;
	}
	return len;
#endif /* _WIN32 */
}

/**
 * Return the length, in codepoints/glyphs, of the utf8-encoded
 * string.  The string is assumed to be at least `len` code-points
 * long. This is needed when splitting words into morphemes.
 */
static inline size_t utf8_strnlen(const char *s, size_t len)
{
	size_t by = 0;
	while (0 < len) { by += utf8_next(&s[by]); }
	return by;
}


/**
 * Copy `n` utf8 characters from `src` to `dest`.
 * Return the number of bytes actually copied.
 * The `dest` must have enough room to hold the copy.
 */
static inline size_t utf8_strncpy(char *dest, const char *src, size_t n)
{
	size_t b = 0;
	while (0 < n)
	{
		size_t k = utf8_next(src);
		b += k;
		while (0 < k) { *dest = *src; dest++; src++; k--; }
		n--;
		if (0x0 == *src) break;
	}

	return b;
}

static inline int is_utf8_upper(const char *s, locale_t dict_locale)
{
	mbstate_t mbs;
	wchar_t c;
	int nbytes;

	memset(&mbs, 0, sizeof(mbs));
	nbytes = mbrtowc(&c, s, MB_CUR_MAX, &mbs);
	if (nbytes < 0) return 0;  /* invalid mb sequence */
	if (iswupper_l(c, dict_locale)) return nbytes;
	return 0;
}

static inline int is_utf8_alpha(const char *s, locale_t dict_locale)
{
	mbstate_t mbs;
	wchar_t c;
	int nbytes;

	memset(&mbs, 0, sizeof(mbs));
	nbytes = mbrtowc(&c, s, MB_CUR_MAX, &mbs);
	if (nbytes < 0) return 0;  /* invalid mb sequence */
	if (iswalpha_l(c, dict_locale)) return nbytes;
	return 0;
}

static inline int is_utf8_digit(const char *s, locale_t dict_locale)
{
	mbstate_t mbs;
	wchar_t c;
	int nbytes;

	memset(&mbs, 0, sizeof(mbs));
	nbytes = mbrtowc(&c, s, MB_CUR_MAX, &mbs);
	if (nbytes < 0) return 0;  /* invalid mb sequence */
	if (iswdigit_l(c, dict_locale)) return nbytes;
	return 0;
}

static inline int is_utf8_space(const char *s, locale_t dict_locale)
{
	mbstate_t mbs;
	wchar_t c;
	int nbytes;

	memset(&mbs, 0, sizeof(mbs));
	nbytes = mbrtowc(&c, s, MB_CUR_MAX, &mbs);
	if (nbytes < 0) return 0;  /* invalid mb sequence */
	if (iswspace_l(c, dict_locale)) return nbytes;

	/* 0xc2 0xa0 is U+00A0, c2 a0, NO-BREAK SPACE */
	/* For some reason, iswspace doesn't get this */
	if ((2==nbytes) && ((0xff & s[0]) == 0xc2) && ((0xff & s[1]) == 0xa0)) return 2;
	if ((2==nbytes) && (c == 0xa0)) return 2;
	return 0;
}

#if 0 /* Not in use. */
static inline const char * skip_utf8_upper(const char * s, locale_t dict_locale)
{
	int nb = is_utf8_upper(s, dict_locale);
	while (nb)
	{
		s += nb;
		nb = is_utf8_upper(s, dict_locale);
	}
	return s;
}

/**
 * Return true if the initial upper-case letters of the
 * two input strings match. Comparison stops when
 * both strings descend to lowercase.
 */
static inline bool utf8_upper_match(const char * s, const char * t,
                                    locale_t dict_locale)
{
	mbstate_t mbs, mbt;
	wchar_t ws, wt;
	int ns, nt;

	memset(&mbs, 0, sizeof(mbs));
	memset(&mbt, 0, sizeof(mbt));

	ns = mbrtowc(&ws, s, MB_CUR_MAX, &mbs);
	nt = mbrtowc(&wt, t, MB_CUR_MAX, &mbt);
	if (ns < 0 || nt < 0) return false;  /* invalid mb sequence */
	while (iswupper_l(ws, dict_locale) || iswupper_l(wt, dict_locale))
	{
		if (ws != wt) return false;
		s += ns;
		t += nt;
		ns = mbrtowc(&ws, s, MB_CUR_MAX, &mbs);
		nt = mbrtowc(&wt, t, MB_CUR_MAX, &mbt);
		if (ns < 0 || nt < 0) return false;  /* invalid mb sequence */
	}
	return true;
}
#endif /* Not in use. */

void downcase_utf8_str(char *to, const char * from, size_t usize, locale_t);
#if 0
void upcase_utf8_str(char *to, const char * from, size_t usize, locale_t);
#endif
int utf8_charlen(const char *);

size_t lg_strlcpy(char * dest, const char *src, size_t size);
void safe_strcpy(char *u, const char * v, size_t usize);
void safe_strcat(char *u, const char *v, size_t usize);
char *safe_strdup(const char *u);

/* Simple, cheap, easy dynamic string. */
typedef struct
{
	char *str;
	size_t end;
	size_t len;
} dyn_str;

dyn_str* dyn_str_new(void);
void dyn_str_delete(dyn_str*);
void dyn_strcat(dyn_str*, const char*);
char * dyn_str_take(dyn_str*);
const char * dyn_str_value(dyn_str*);

size_t altlen(const char **);

/* routines for allocating basic objects */
void init_memusage(void);
void * xalloc(size_t) GNUC_MALLOC;
void * exalloc(size_t) GNUC_MALLOC;

/* Tracking the space usage can help with debugging */
#ifdef TRACK_SPACE_USAGE
void xfree(void *, size_t);
void exfree(void *, size_t);
#else /* TRACK_SPACE_USAGE */
static inline void xfree(void *p, size_t sz) { free(p); }
static inline void exfree(void *p, size_t sz) { free(p); };
#endif /* TRACK_SPACE_USAGE */

size_t get_space_in_use(void);
size_t get_max_space_used(void);


char * get_default_locale(void);
void set_utf8_program_locale(void);
bool try_locale(const char *);

/**
 * Returns the smallest power of two that is at least i and at least 1
 */
static inline unsigned int next_power_of_two_up(unsigned int i)
{
	unsigned int j=1;
	while (j<i) j <<= 1;
	return j;
}

#endif /* _LINK_GRAMMAR_UTILITIES_H_ */

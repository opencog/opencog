/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2008, 2009, 2013 Linas Vepstas                              */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <ctype.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <stdarg.h>
#include <locale.h>
#ifdef HAVE_LOCALE_T_IN_XLOCALE_H
#include <xlocale.h>
#endif /* HAVE_LOCALE_T_IN_XLOCALE_H */

#ifndef _WIN32
	// #include <unistd.h>
	#include <langinfo.h>
#else
	#include <windows.h>
#endif /* _WIN32 */

#include "utilities.h"

/* This file contains general utilities that fix, enhance OS-provided
 * API's, esp ones that the OS forgot to provide, or managed to break.
 */

/* ============================================================= */
/* String utilities */

char *safe_strdup(const char *u)
{
	if (u)
		return strdup(u);
	return NULL;
}

/**
 * Copies as much of v into u as it can assuming u is of size usize
 * guaranteed to terminate u with a '\0'.
 */
void safe_strcpy(char *u, const char * v, size_t usize)
{
	strncpy(u, v, usize-1);
	u[usize-1] = '\0';
}

/**
 * A version of strlcpy, for those systems that don't have it.
 */
size_t lg_strlcpy(char * dest, const char *src, size_t size)
{
	size_t i=0;
	while ((i<size) && (src[i] != 0x0))
	{
		dest[i] = src[i];
		i++;
	}
	if (i < size) { dest[i] = 0x0; size = i; }
	else if (0 < size) { size --; dest[size] = 0x0;}
	return size;
}

/**
 * Catenates as much of v onto u as it can assuming u is of size usize
 * guaranteed to terminate u with a '\0'.  Assumes u and v are null
 * terminated.
 */
void safe_strcat(char *u, const char *v, size_t usize)
{
	strncat(u, v, usize-strlen(u)-1);
	u[usize-1] = '\0';
}

#ifndef HAVE_STRNDUP
/* Emulates glibc's strndup() */
char *
strndup (const char *str, size_t size)
{
	size_t len;
	char *result = (char *) NULL;

	if ((char *) NULL == str) return (char *) NULL;

	len = strlen (str);
	if (!len) return strdup ("");
	if (size > len) size = len;

	result = (char *) malloc ((size + 1) * sizeof (char));
	memcpy (result, str, size);
	result[size] = 0x0;
	return result;
}
#endif /* !HAVE_STRNDUP */

#ifndef HAVE_STRTOK_R
/*
 * public domain strtok_r() by Charlie Gordon
 * from comp.lang.c  9/14/2007
 *     http://groups.google.com/group/comp.lang.c/msg/2ab1ecbb86646684
 *
 *     Declaration that it's public domain:
 *     http://groups.google.com/group/comp.lang.c/msg/7c7b39328fefab9c
 */
char* strtok_r(char *str, const char *delim, char **nextp)
{
	char *ret;

	if (str == NULL) str = *nextp;
	str += strspn(str, delim);
	if (*str == '\0') return NULL;
	ret = str;
	str += strcspn(str, delim);
	if (*str) *str++ = '\0';
	*nextp = str;

	return ret;
}
#endif /* !HAVE_STRTOK_R */

/* ============================================================= */
/* UTF8 utilities */

/** Returns length of UTF8 character.
 * Current algo is based on the first character only.
 * If pointer is not pointing at first char, no not a valid value, returns 0.
 * Returns 0 for NULL as well.
 */
int utf8_charlen(const char *xc)
{
	unsigned char c;

	c = (unsigned char) *xc;

	if (c == 0) return 0;
	if (c < 0x80) return 1;
	if ((c >= 0xc2) && (c < 0xe0)) return 2; /* First byte of a code point U +0080 - U +07FF */
	if ((c >= 0xe0) && (c < 0xf0)) return 3; /* First byte of a code point U +0800 - U +FFFF */
	if ((c >= 0xf0) && (c <= 0xf4)) return 4; /* First byte of a code point U +10000 - U +10FFFF */
	return -1; /* Fallthrough -- not the first byte of a code-point. */
}

#ifdef _WIN32
/**
 * (Experimental) Implementation of mbrtowc for Windows.
 * This is required because the other, commonly available implementations
 * seem to not work very well, based on user reports.  Someone who is
 * really, really good at windows programming needs to review this stuff!
 */
size_t lg_mbrtowc(wchar_t *pwc, const char *s, size_t n, mbstate_t *ps)
{
	int nb, nb2;

	if (NULL == s) return 0;
	if (0 == n) return -2;
	if (0 == *s) { *pwc = 0; return 0; }

	nb = utf8_charlen(s);
	if (0 == nb) return 0;
	if (0 > nb) return nb;
	nb2 = MultiByteToWideChar(CP_UTF8, 0, s, nb, NULL, 0);
	nb2 = MultiByteToWideChar(CP_UTF8, 0, s, nb, pwc, nb2);
	if (0 == nb2) return (size_t)-1;
	return nb;
}
#endif /* _WIN32 */

static int wctomb_check(char *s, wchar_t wc)
{
	int nr;
#ifdef _WIN32
	nr = WideCharToMultiByte(CP_UTF8, 0, &wc, 1, NULL, 0, NULL, NULL);
	nr = WideCharToMultiByte(CP_UTF8, 0, &wc, 1, s, nr, NULL, NULL);
	if (0 == nr) return -1;
#else
	mbstate_t mbss;
	memset(&mbss, 0, sizeof(mbss));
	nr = wcrtomb(s, wc, &mbss);
	if (nr < 0) {
		prt_error("Fatal Error: unknown character set %s\n", nl_langinfo(CODESET));
		exit(1);
	}
#endif /* _WIN32 */
	return nr;
}

/**
 * Downcase the first letter of the word.
 * XXX FIXME This works 'most of the time', but is not technically correct.
 * This is because towlower() and towupper() are locale dependent, and also
 * because the byte-counts might not match up, e.g. German ß and SS.
 * The correct long-term fix is to use ICU or glib g_utf8_strup(), etc.
 */
void downcase_utf8_str(char *to, const char * from, size_t usize, locale_t locale_t)
{
	wchar_t c;
	int i, nbl, nbh;
	char low[MB_LEN_MAX];
	mbstate_t mbs;

	/* Make sure it doesn't contain garbage in case of an error */
	if (to != from) strcpy(to, from);

	memset(&mbs, 0, sizeof(mbs));
	nbh = mbrtowc (&c, from, MB_CUR_MAX, &mbs);
	if (nbh < 0)
	{
		prt_error("Error: Invalid UTF-8 string!\n");
		return;
	}
	c = towlower_l(c, locale_t);
	nbl = wctomb_check(low, c);

	/* Check for error on an in-place copy */
	if ((nbh < nbl) && (to == from))
	{
		/* I'm to lazy to fix this */
		prt_error("Error: can't downcase UTF-8 string!\n");
		return;
	}

	/* Downcase */
	for (i=0; i<nbl; i++) { to[i] = low[i]; }

	if ((nbh == nbl) && (to == from)) return;

	from += nbh;
	to += nbl;
	safe_strcpy(to, from, usize-nbl);
}

#if 0
/**
 * Upcase the first letter of the word.
 * XXX FIXME This works 'most of the time', but is not technically correct.
 * This is because towlower() and towupper() are locale dependent, and also
 * because the byte-counts might not match up, e.g. German ß and SS.
 * The correct long-term fix is to use ICU or glib g_utf8_strup(), etc.
 */
void upcase_utf8_str(char *to, const char * from, size_t usize, locale_t locale_t)
{
	wchar_t c;
	int i, nbl, nbh;
	char low[MB_LEN_MAX];
	mbstate_t mbs;

	memset(&mbs, 0, sizeof(mbs));
	nbh = mbrtowc (&c, from, MB_CUR_MAX, &mbs);
	if (nbh < 0)
	{
		prt_error("Error: Invalid UTF-8 string!\n");
		return;
	}
	c = towupper_l(c, locale_t);
	nbl = wctomb_check(low, c);

	/* Check for error on an in-place copy */
	if ((nbh < nbl) && (to == from))
	{
		/* I'm to lazy to fix this */
		prt_error("Error: can't upcase UTF-8 string!\n");
		return;
	}

	/* Upcase */
	for (i=0; i<nbl; i++) { to[i] = low[i]; }

	if ((nbh == nbl) && (to == from)) return;

	from += nbh;
	to += nbl;
	safe_strcpy(to, from, usize-nbl);
}
#endif

/* ============================================================= */
/* Memory alloc routines below. These routines attempt to keep
 * track of how much space is getting used during a parse.
 *
 * This code is probably obsolescent, and should probably be dumped.
 * No one (that I know of) looks at the space usage; its one of the
 * few areas that needs pthreads -- it would be great to just get
 * rid of it (and thus get rid of pthreads).
 */

#ifdef TRACK_SPACE_USAGE
typedef struct
{
	size_t max_space_used;
	size_t space_in_use;
	size_t num_xallocs;
	size_t num_xfrees;
	size_t max_outstanding_xallocs;
	size_t max_external_space_used;
	size_t external_space_in_use;
	size_t num_exallocs;
	size_t num_exfrees;
	size_t max_outstanding_exallocs;
} space_t;

static TLS space_t space;
static space_t * do_init_memusage(void)
{
	space_t *s = &space;

	s->max_space_used = 0;
	s->space_in_use = 0;
	s->num_xallocs = 0;
	s->num_xfrees = 0;
	s->max_outstanding_xallocs = 0;
	s->max_external_space_used = 0;
	s->external_space_in_use = 0;
	s->num_exallocs = 0;
	s->num_exfrees = 0;
	s->max_outstanding_exallocs = 0;

	return s;
}

void init_memusage(void)
{
	static bool mem_inited = false;
	if (mem_inited) return;
	mem_inited = true;
	do_init_memusage();
}

static inline space_t *getspace(void)
{
	return &space;
}

/**
 * space used but not yet freed during parse
 */
size_t get_space_in_use(void)
{
	return getspace()->space_in_use;
}

/**
 * maximum space used during the parse
 */
size_t get_max_space_used(void)
{
	return getspace()->max_space_used;
}
#else /* TRACK_SPACE_USAGE */
void init_memusage(void) {}
size_t get_space_in_use(void) { return 0; }
size_t get_max_space_used(void) { return 0; }
#endif /* TRACK_SPACE_USAGE */

/**
 * alloc some memory, and keep track of the space allocated.
 */
void * xalloc(size_t size)
{
	void * p = malloc(size);

#ifdef TRACK_SPACE_USAGE
	space_t *s = getspace();
	s->space_in_use += size;
	if (s->max_space_used < s->space_in_use) s->max_space_used = s->space_in_use;
	s->num_xallocs ++;
	if (s->max_outstanding_xallocs < (s->num_xallocs - s->num_xfrees))
		s->max_outstanding_xallocs = (s->num_xallocs - s->num_xfrees);

#endif /* TRACK_SPACE_USAGE */
	if ((p == NULL) && (size != 0))
	{
		prt_error("Fatal Error: Ran out of space. (int)\n");
		abort();
		exit(1);
	}
	return p;
}

#ifdef TRACK_SPACE_USAGE
void xfree(void * p, size_t size)
{
	space_t *s = getspace();
	s->space_in_use -= size;
	s->num_xfrees ++;

	free(p);
}
#endif /* TRACK_SPACE_USAGE */

void * exalloc(size_t size)
{
	void * p = malloc(size);
#ifdef TRACK_SPACE_USAGE
	space_t *s = getspace();
	s->external_space_in_use += size;
	if (s->max_external_space_used < s->external_space_in_use)
		s->max_external_space_used = s->external_space_in_use;
	s->num_exallocs ++;
	if (s->max_outstanding_exallocs < (s->num_exallocs - s->num_exfrees))
		s->max_outstanding_exallocs = (s->num_exallocs - s->num_exfrees);
#endif /* TRACK_SPACE_USAGE */

	if ((p == NULL) && (size != 0))
	{
		prt_error("Fatal Error: Ran out of space. (ext)\n");
		abort();
		exit(1);
	}
	return p;
}

#ifdef TRACK_SPACE_USAGE
void exfree(void * p, size_t size)
{
	space_t *s = getspace();
	s->external_space_in_use -= size;
	s->num_exfrees ++;
	free(p);
}
#endif /* TRACK_SPACE_USAGE */

/* =========================================================== */
/* Simple, cheap, easy dynamic string. */

dyn_str* dyn_str_new(void)
{
	dyn_str *ds = malloc(sizeof(dyn_str));
	ds->len = 250;
	ds->end = 0;
	ds->str = malloc(ds->len);
	ds->str[0] = 0x0;
	return ds;
}

void dyn_str_delete(dyn_str* ds)
{
	free(ds->str);
	free(ds);
}

char * dyn_str_take(dyn_str* ds)
{
	char * rv = ds->str;
	free(ds);
	return rv;
}

void dyn_strcat(dyn_str* ds, const char *str)
{
	size_t l = strlen(str);
	if (ds->end+l+1 >= ds->len)
	{
		ds->len = 2 * ds->len + l;
		ds->str = realloc(ds->str, ds->len);
	}
	strcpy (ds->str+ds->end, str);
	ds->end += l;
}

const char * dyn_str_value(dyn_str* s)
{
	return s->str;
}

/* ======================================================== */
/* Locale routines */

#ifdef HAVE_LOCALE_T
/**
 * Create a locale object from the given locale string.
 * @param locale Locale string, in the native OS format.
 * @return Locale object for the given locale
 * Note: It has to be freed by freelocale().
 */
locale_t newlocale_LC_CTYPE(const char *locale)
{
	locale_t locobj;
#ifdef _WIN32
	locobj = _create_locale(LC_CTYPE, locale);
#else
	locobj = newlocale(LC_CTYPE_MASK, locale, (locale_t)0);
#endif /* _WIN32 */
	return locobj;
}
#endif /* HAVE_LOCALE_T */

/**
 * Check that the given locale known by the system.
 * In case we don't have locale_t, actually set the locale
 * in order to find out if it is fine. This side effect doesn't cause
 * harm, as the locale would be set up to that value anyway shortly.
 * @param locale Locale string
 * @return True if known, false if unknown.
 */
bool try_locale(const char *locale)
{
#ifdef HAVE_LOCALE_T
		locale_t ltmp = newlocale_LC_CTYPE(locale);
		if ((locale_t)0 == ltmp) return false;
		freelocale(ltmp);
#else
		lgdebug(D_USER_FILES, "Debug: Setting program's locale \"%s\"", locale);
		if (NULL == setlocale(LC_CTYPE, locale))
		{
			lgdebug(D_USER_FILES, " failed!\n");
			return false;
		}
		lgdebug(D_USER_FILES, ".\n");
#endif /* HAVE_LOCALE_T */

		return true;
}

/**
 * Ensure that the program's locale has a UTF-8 codeset.
 */
void set_utf8_program_locale(void)
{
#ifndef _WIN32
	/* The LG library doesn't use mbrtowc_l(), since it doesn't exist in
	 * the dynamic glibc (2.22). mbsrtowcs_l() could also be used, but for
	 * some reason it exists only in the static glibc.
	 * In order that mbrtowc() will work for any UTF-8 character, UTF-8
	 * codeset is ensured. */
	const char *codeset = nl_langinfo(CODESET);
	if (!strstr(codeset, "UTF") && !strstr(codeset, "utf"))
	{
		const char *locale = setlocale(LC_CTYPE, NULL);
		/* Avoid an initial spurious message. */
		if ((0 != strcmp(locale, "C")) && (0 != strcmp(locale, "POSIX")))
		{
			prt_error("Warning: Program locale \"%s\" (codeset %s) was not UTF-8; "
						 "force-setting to en_US.UTF-8\n", locale, codeset);
		}
		locale = setlocale(LC_CTYPE, "en_US.UTF-8");
		if (NULL == locale)
		{
			prt_error("Warning: Program locale en_US.UTF-8 could not be set; "
			          "force-setting to C.UTF-8\n");
			locale = setlocale(LC_CTYPE, "C.UTF-8");
			if (NULL == locale)
			{
				prt_error("Warning: Could not set a UTF-8 program locale; "
				          "program may malfunction\n");
			}
		}
	}
#endif /* !_WIN32 */
}

#ifdef _WIN32
static char *
win32_getlocale (void)
{
	char lbuf[10];
	char locale[32];

	LCID lcid = GetThreadLocale();

	if (0 >= GetLocaleInfoA(lcid, LOCALE_SISO639LANGNAME, lbuf, sizeof(lbuf)))
	{
		prt_error("Error: GetLocaleInfoA LOCALE_SENGLISHLANGUAGENAME LCID=%d: "
		          "Error %d\n", (int)lcid, (int)GetLastError());
		return NULL;
	}
	strcpy(locale, lbuf);
	strcat(locale, "-");

	if (0 >= GetLocaleInfoA(lcid, LOCALE_SISO3166CTRYNAME, lbuf, sizeof(lbuf)))
	{
		prt_error("Error: GetLocaleInfoA LOCALE_SISO3166CTRYNAME LCID=%d: "
		          "Error %d\n", (int)lcid, (int)GetLastError());
		return NULL;
	}
	strcat(locale, lbuf);

	return strdup(locale);
}
#endif /* _WIN32 */

char * get_default_locale(void)
{
	const char *lc_vars[] = {"LC_ALL", "LC_CTYPE", "LANG", NULL};
	char *ev;
	const char **evname;
	char *locale = NULL;

	for(evname = lc_vars; NULL != *evname; evname++)
	{
		ev = getenv(*evname);
		if ((NULL != ev) && ('\0' != ev[0])) break;
	}
	if (NULL != *evname)
	{
		locale = ev;
		lgdebug(D_USER_FILES, "Debug: Environment locale \"%s=%s\"\n", *evname, ev);
#ifdef _WIN32
		/* If compiled with MSVC/MinGW, we still support running under Cygwin. */
		const char *ostype = getenv("OSTYPE");
		if ((NULL != ostype) && (0 == strcmp(ostype, "cygwin")))
		{
			/* Convert to Windows style locale */
			locale = strdupa(locale);
			locale[strcspn(locale, "_")] = '-';
			locale[strcspn(locale, ".@")] = '\0';
		}
#endif /* _WIN32 */
	}
	else
	{
		lgdebug(D_USER_FILES, "Debug: Environment locale not set\n");
#ifdef _WIN32
		locale = win32_getlocale();
		if (NULL == locale)
			lgdebug(D_USER_FILES, "Debug: Cannot find user default locale\n");
		else
			lgdebug(D_USER_FILES, "Debug: User default locale \"%s\"\n", locale);
		return locale; /* Already strdup'ed */
#endif /* _WIN32 */
	}

	return safe_strdup(locale);
}

/* ============================================================= */
/* Alternatives utilities */

size_t altlen(const char **arr)
{
	size_t len = 0;
	if (arr)
		while (arr[len] != NULL) len++;
	return len;
}

/* ============================================================= */

#ifdef __MINGW32__
/*
 * Since _USE_MINGW_ANSI_STDIO=1 is used in order to support C99 STDIO
 * including the %z formats, MinGW uses its own *printf() functions (and not
 * the Windows ones). However, its printf()/fprintf() functions cannot write
 * UTF-8 to the console (to files/pipes they write UTF-8 just fine).  It
 * turned out the problem is that they use the putchar() of Windows, which
 * doesn't support writing UTF-8 only when writing to the console!  This
 * problem is not fixed even in Windows 10 and the latest MinGW in Cygwin
 * 2.5.2.
 *
 * The workaround implemented here is to reimplement the corresponding MinGW
 * internal functions, and use fputs() to write the result.
 *
 * (Reimplementing printf()/fprintf() this way didn't work even with the
 * compilation flag -fno-builtin .)
 */

int __mingw_vfprintf (FILE * __restrict__ stream, const char * __restrict__ fmt,
                      va_list vl)
{
	int n = vsnprintf(NULL, 0, fmt, vl);
	if (0 > n) return n;
	char *buf = malloc(n+1);
	n = vsnprintf(buf, n+1, fmt, vl);
	if (0 > n)
	{
		free(buf);
		return n;
	}

	n = fputs(buf, stdout);
	free(buf);
	return n;
}

int __mingw_vprintf (const char * __restrict__ fmt, va_list vl)
{
	return __mingw_vfprintf(stdout, fmt, vl);
}
#endif /* __MINGW32__ */
/* ============================================================= */

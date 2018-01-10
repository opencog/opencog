/***************************************************************************/
/* Copyright (c) 2016 Amir Plivatsky                                       */
/* lg_isatty() is based on code sent to the Cygwin discussion group by     */
/* Corinna Vinschen, Cygwin Project Co-Leader, on 2012.                    */
/* All rights reserved                                                     */
/*                                                                         */
/* Use of the link grammar parsing system is subject to the terms of the   */
/* license set forth in the LICENSE file included with this software.      */
/* This license allows free redistribution and use in source and binary    */
/* forms, with or without modification, subject to certain conditions.     */
/*                                                                         */
/***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <io.h>
#ifdef __MINGW32__
#include <malloc.h>
#endif /* __MINGW32__ */
#include <errno.h>

#include "parser-utilities.h"

/**
 * Get a line from the console in UTF-8.
 * This function bypasses the code page conversion and reads Unicode
 * directly from the console.
 * @return An input line from the console in UTF-8 encoding.
 */
char *get_console_line(void)
{
	static HANDLE console_handle = NULL;
	wchar_t winbuf[MAX_INPUT];
	/* Worst-case: 4 bytes per UTF-8 char, 1 UTF-8 char per wchar_t char. */
	static char utf8inbuf[MAX_INPUT*4+1];

	if (NULL == console_handle)
	{
		console_handle = CreateFileA("CONIN$", GENERIC_READ, FILE_SHARE_READ,
		                             NULL, OPEN_EXISTING, 0, NULL);
		if (!console_handle || (INVALID_HANDLE_VALUE == console_handle))
		{
			prt_error("Error: CreateFileA CONIN$: Error %d.\n", (int)GetLastError());
			return NULL;
		}
	}

	DWORD nchar;
	if (!ReadConsoleW(console_handle, &winbuf, MAX_INPUT-sizeof(wchar_t), &nchar, NULL))
	{
		prt_error("Error: ReadConsoleW: Error %d\n", (int)GetLastError());
		return NULL;
	}
	winbuf[nchar] = L'\0';

	nchar = WideCharToMultiByte(CP_UTF8, 0, winbuf, -1, utf8inbuf,
	                            sizeof(utf8inbuf), NULL, NULL);
	if (0 == nchar)
	{
		prt_error("Error: WideCharToMultiByte CP_UTF8 failed: Error %d.\n",
		          (int)GetLastError());
		return NULL;
	}

	/* Make sure we don't have conversion problems, by searching for '�'. */
	const char *invlid_char  = strstr(utf8inbuf, "\xEF\xBF\xBD");
	if (NULL != invlid_char)
		prt_error("Warning: Invalid input character encountered.\n");

	/* ^Z is read as a character. Convert it to an EOF indication. */
	if ('\x1A' == utf8inbuf[0]) /* Only handle it at line start. */
		return NULL;

	return utf8inbuf;
}

static int console_input_cp;
static int console_output_cp;
static void restore_console_cp(void)
{
	SetConsoleCP(console_input_cp);
	SetConsoleOutputCP(console_output_cp);
}

static BOOL CtrlHandler(DWORD fdwCtrlType)
{
	if ((CTRL_C_EVENT == fdwCtrlType) || (CTRL_BREAK_EVENT  == fdwCtrlType))
	{
		fprintf(stderr, "Interrupt\n");
		restore_console_cp();
		exit(2);
	}
	return FALSE;
}

/**
 * Set the output conversion attributes for transparency.
 * This way UTF-8 output doesn't pass any conversion.
 */
void win32_set_utf8_output(void)
{
	if (-1 == _setmode(fileno(stdout), _O_BINARY))
	{
		prt_error("Warning: _setmode(fileno(stdout), _O_BINARY): %s.\n",
			strerror(errno));
	}

	console_input_cp = GetConsoleCP();
	console_output_cp = GetConsoleOutputCP();
	atexit(restore_console_cp);
	if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE))
	{
		prt_error("Warning: Cannot not set code page restore handler.\n");
	}
	/* For file output. It is too late for output pipes.
	 * If output pipe is desired, one can set CP_UTF8 by the
	 * command "chcp 65001" before invoking link-parser. */
	if (!SetConsoleCP(CP_UTF8))
	{
		prt_error("Warning: Cannot set input codepage %d (error %d).\n",
			CP_UTF8, (int)GetLastError());
	}
	/* For Console output. */
	if (!SetConsoleOutputCP(CP_UTF8))
	{
		prt_error("Warning: Cannot set output codepage %d (error %d).\n",
			CP_UTF8, (int)GetLastError());
	}
}

#include <winternl.h>
/**
 * isatty() compatibility for running under Cygwin when compiling
 * using the Windows native C library.
 * WIN32 isatty() gives a wrong result (for Cygwin) on Cygwin's
 * pseudo-ttys, because they are implemented as Windows pipes.
 * Here is compatibility layer, originally sent to the Cygwin
 * discussion group by Corinna Vinschen, Cygwin Project Co-Leader.
 * See: https://www.cygwin.com/ml/cygwin/2012-11/msg00214.html
 */
int lg_isatty(int fd)
{
	HANDLE fh;
	long buf[66];  /* NAME_MAX + 1 + sizeof ULONG */
	PFILE_NAME_INFO pfni = (PFILE_NAME_INFO)buf;
	PWCHAR cp;

	/* Fetch the underlying HANDLE. */
	fh = (HANDLE)_get_osfhandle(fd);
	if (!fh || (INVALID_HANDLE_VALUE == fh))
	{
		errno = EBADF;
		return 0;
	}

/* Windows _isatty() is buggy: It returns a nonzero for the NUL device! */
#if 0
	if (_isatty(fd))
		return 1;
#else
	/* This detects a console device reliably. */
	CONSOLE_SCREEN_BUFFER_INFO sbi;
	DWORD mode;
	if (GetConsoleMode(fh, &mode) || GetConsoleScreenBufferInfo(fh, &sbi))
		return 1;
#endif

	/* Must be a pipe. */
	if (GetFileType(fh) != FILE_TYPE_PIPE)
		goto no_tty;

	if (!GetFileInformationByHandleEx(fh, FileNameInfo, pfni, sizeof(buf)))
	{
		printf("GetFileInformationByHandleEx: Error %d\n", (int)GetLastError());
		goto no_tty;
	}

	/* The filename is not guaranteed to be NUL-terminated. */
	pfni->FileName[pfni->FileNameLength / sizeof (WCHAR)] = L'\0';

	/* Now check the name pattern.  The filename of a Cygwin pseudo tty pipe
	   looks like this:

			 \cygwin-%16llx-pty%d-{to,from}-master

		%16llx is the hash of the Cygwin installation, (to support multiple
		parallel installations), %d id the pseudo tty number, "to" or "from"
		differs the pipe direction. "from" is a stdin, "to" a stdout-like
		pipe. */
	cp = pfni->FileName;
	if (!wcsncmp(cp, L"\\cygwin-", 8) && !wcsncmp(cp + 24, L"-pty", 4))
	{
		cp = wcschr(cp + 28, '-');
		if (!cp)
			goto no_tty;
		if (!wcscmp(cp, L"-from-master") || !wcscmp(cp, L"-to-master"))
			return 1;
	}
no_tty:
	errno = EINVAL;
	return 0;
}
#endif /* _WIN32 */

#ifdef __MINGW32__
/*
 * A workaround for printing UTF-8 on the console.
 * These functions are also implemented in the LG library.
 * For the strange story see the comment in utilities.c there.
 */

int __mingw_vfprintf (FILE * __restrict__ stream, const char * __restrict__ fmt, va_list vl)
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

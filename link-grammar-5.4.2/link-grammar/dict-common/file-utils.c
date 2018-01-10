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

#include <sys/types.h>
#include <sys/stat.h>   /* for fstat() */

#ifndef _WIN32
	#include <unistd.h>
#else
	#include <windows.h>
	#include <Shlwapi.h> /* For PathRemoveFileSpecA(). */
	#include <direct.h>  /* For getcwd(). */
#endif /* _WIN32 */

#include <stdlib.h>
#include <string.h>

#include <link-includes.h>

#include "file-utils.h"
#include "error.h"  // for verbosity_level
#include "utilities.h"

#ifdef _WIN32
	#define DIR_SEPARATOR "\\"
#else
	#define DIR_SEPARATOR "/"
#endif /*_WIN32 */

#define IS_DIR_SEPARATOR(ch) (DIR_SEPARATOR[0] == (ch))
#if !defined(DICTIONARY_DIR) || defined(__MINGW32__)
	#define DEFAULTPATH NULL
#else
	#define DEFAULTPATH DICTIONARY_DIR
#endif

/* =========================================================== */
/* File path and dictionary open routines below */

#define MAX_PATH_NAME 200     /* file names (including paths)
                                 should not be longer than this */

char * join_path(const char * prefix, const char * suffix)
{
	char * path;
	size_t path_len, prel;

	path_len = strlen(prefix) + 1 /* len(DIR_SEPARATOR) */ + strlen(suffix);
	path = (char *) malloc(path_len + 1);

	strcpy(path, prefix);

	/* Windows is allergic to multiple path separators, so append one
	 * only if the prefix isn't already terminated by a path sep.
	 */
	prel = strlen(path);
	if (0 < prel && path[prel-1] != DIR_SEPARATOR[0])
	{
		path[prel] = DIR_SEPARATOR[0];
		path[prel+1] = '\0';
	}
	strcat(path, suffix);

	return path;
}

/* global - but that's OK, since this is set only during initialization,
 * and is is thenceforth a read-only item. So it doesn't need to be
 * locked.
 */
static char * custom_data_dir = NULL;

void dictionary_set_data_dir(const char * path)
{
	if (custom_data_dir) free (custom_data_dir);
	custom_data_dir = safe_strdup(path);
}

char * dictionary_get_data_dir(void)
{
	char * data_dir = NULL;

	if (custom_data_dir != NULL) {
		data_dir = safe_strdup(custom_data_dir);
		return data_dir;
	}

#ifdef _WIN32
	/* Dynamically locate invocation directory of our program.
	 * Non-ASCII characters are not supported (files will not be found). */
	char prog_path[MAX_PATH_NAME];

	if (!GetModuleFileNameA(NULL, prog_path, sizeof(prog_path)))
	{
		prt_error("Warning: GetModuleFileName error %d\n", (int)GetLastError());
	}
	else
	{
		if (NULL == prog_path)
		{
			/* Can it happen? */
			prt_error("Warning: GetModuleFileName returned a NULL program path!\n");
		}
		else
		{
			if (!PathRemoveFileSpecA(prog_path))
			{
				prt_error("Warning: Cannot get directory from program path '%s'!\n",
				          prog_path);
			}
			else
			{
				/* Unconvertible characters are marked as '?' */
				const char *unsupported = (NULL != strchr(prog_path, '?')) ?
					" (containing unsupported character)" : "";

				lgdebug(D_USER_FILES, "Debug: Directory of executable: %s%s\n",
				        unsupported, prog_path);
				data_dir = safe_strdup(prog_path);
			}
		}
	}
#endif /* _WIN32 */

	return data_dir;
}

/**
 * Locate a data file and open it.
 *
 * This function is used to open a dictionary file or a word file,
 * or any associated data file (like a post process knowledge file).
 *
 * It works as follows.  If the file name begins with a "/", then
 * it's assumed to be an absolute file name and it tries to open
 * that exact file.
 *
 * Otherwise, it looks for the file in a sequence of directories, as
 * specified in the dictpath array, until it finds it.
 *
 * If it is still not found, it may be that the user specified a relative
 * path, so it tries to open the exact file.
 *
 * Associated data files are looked in the *same* directory in which the
 * first one was found (typically "en/4.0.dict").  The private static
 * "path_found" serves as a directory path cache which records where the
 * first file was found.  The goal here is to avoid insanity due to
 * user's fractured installs.
 * If the filename argument is NULL, the function just invalidates this
 * directory path cache.
 */
#define NOTFOUND(fp) ((NULL == (fp)) ? " (Not found)" : "")
void * object_open(const char *filename,
                   void * (*opencb)(const char *, const void *),
                   const void * user_data)
{
	static char *path_found; /* directory path cache */
	char *completename = NULL;
	void *fp = NULL;
	char *data_dir = NULL;
	const char **path = NULL;

	if (NULL == filename)
	{
		/* Invalidate the directory path cache */
		free(path_found);
		path_found = NULL;
		return NULL;
	}

	if (NULL == path_found)
	{
		data_dir = dictionary_get_data_dir();
		if (verbosity_level(D_USER_FILES))
		{
			char cwd[MAX_PATH_NAME];
			char *cwdp = getcwd(cwd, sizeof(cwd));
			prt_error("Debug: Current directory: %s\n", NULL == cwdp ? "NULL": cwdp);
			prt_error("Debug: Last-resort data directory: %s\n",
					  data_dir ? data_dir : "NULL");
		}
	}

	/* Look for absolute filename.
	 * Unix: starts with leading slash.
	 * Windows: starts with C:\  except that the drive letter may differ.
	 * Note that only native windows C library uses backslashes; mingw
	 * seems to use forward-slash, from what I can tell.
	 */
	if ((filename[0] == '/')
#ifdef _WIN32
		|| ((filename[1] == ':')
			 && ((filename[2] == '\\') || (filename[2] == '/')))
		|| (filename[0] == '\\') /* UNC path */
#endif /* _WIN32 */
	   )
	{
		/* opencb() returns NULL if the file does not exist. */
		fp = opencb(filename, user_data);
		lgdebug(D_USER_FILES, "Debug: Opening file %s%s\n", filename, NOTFOUND(fp));
	}
	else
	{
		/* A path list in which to search for dictionaries.
		 * path_found, data_dir or DEFAULTPATH may be NULL. */
		const char *dictpath[] =
		{
			path_found,
			".",
			"." DIR_SEPARATOR "data",
			"..",
			".." DIR_SEPARATOR "data",
			data_dir,
			DEFAULTPATH,
		};
		size_t i = sizeof(dictpath)/sizeof(dictpath[0]);

		for (path = dictpath; i-- > 0; path++)
		{
			if (NULL == *path) continue;

			free(completename);
			completename = join_path(*path, filename);
			fp = opencb(completename, user_data);
			lgdebug(D_USER_FILES, "Debug: Opening file %s%s\n", completename, NOTFOUND(fp));
			if ((NULL != fp) || (NULL != path_found)) break;
		}
	}

	if (NULL == fp)
	{
		fp = opencb(filename, user_data);
		lgdebug(D_USER_FILES, "Debug: Opening file %s%s\n", filename, NOTFOUND(fp));
	}
	else if (NULL == path_found)
	{
		size_t i;

		path_found = strdup((NULL != completename) ? completename : filename);
		if (0 < verbosity)
			prt_error("Info: Dictionary found at %s\n", path_found);
		for (i = 0; i < 2; i++)
		{
			char *root = strrchr(path_found, DIR_SEPARATOR[0]);
			if (NULL != root) *root = '\0';
		}
	}

	free(data_dir);
	free(completename);
	return fp;
}
#undef NOTFOUND

static void *dict_file_open(const char *fullname, const void *how)
{
	return fopen(fullname, how);
}

FILE *dictopen(const char *filename, const char *how)
{
	return object_open(filename, dict_file_open, how);
}

/* ======================================================== */

/**
 * Check to see if a file exists.
 */
bool file_exists(const char * dict_name)
{
	bool retval = false;
	int fd;
	struct stat buf;

	/* On Windows, 'b' (binary mode) is mandatory, otherwise fstat file length
	 * is confused by crlf counted as one byte. POSIX systems just ignore it. */
	FILE *fp = dictopen(dict_name, "rb");

	if (fp == NULL)
		return false;

	/* Get the file size, in bytes. */
	fd = fileno(fp);
	fstat(fd, &buf);
	if (0 < buf.st_size) retval = true;

	fclose(fp);
	return retval;
}

/**
 * Read in the whole stinkin file. This routine returns
 * malloced memory, which should be freed as soon as possible.
 */
char *get_file_contents(const char * dict_name)
{
	int fd;
	size_t tot_size;
	int left;
	struct stat buf;
	char * contents, *p;

	/* On Windows, 'b' (binary mode) is mandatory, otherwise fstat file length
	 * is confused by crlf counted as one byte. POSIX systems just ignore it. */
	FILE *fp = dictopen(dict_name, "rb");

	if (fp == NULL)
		return NULL;

	/* Get the file size, in bytes. */
	fd = fileno(fp);
	fstat(fd, &buf);
	tot_size = buf.st_size;

	contents = (char *) malloc(sizeof(char) * (tot_size+7));

	/* Now, read the whole file. */
	p = contents;
	*p = '\0';
	left = tot_size + 7;
	while (1)
	{
		char *rv = fgets(p, left, fp);
		if (NULL == rv || feof(fp))
			break;
		while (*p != '\0') { p++; left--; }
		if (left < 0)
			 break;
	}

	fclose(fp);

	if (left < 0)
	{
		prt_error("Error: File size is insane!\n");
		free(contents);
		return NULL;
	}

	return contents;
}

/* ============================================================= */

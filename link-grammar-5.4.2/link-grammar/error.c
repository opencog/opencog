/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2009 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "error.h"
#include "api-structures.h"   // For Sentence_s
#include "print/print.h"      // For print_sentence_context()
#include "print/print-util.h" // For append_string()

static void default_error_handler(lg_errinfo *, void *);
static TLS struct
{
	lg_error_handler handler;
	void *handler_data;
	lg_errinfo *errmsg;
} lg_error = { default_error_handler };

/* This list should match enum lg_error_severity. */
#define MAX_SEVERITY_LABEL_SIZE 64 /* In bytes. */
const char *severity_label_by_level[] =
{
	"Fatal error", "Error", "Warning", "Info", "Debug", "Trace", /*lg_None*/"",
	NULL
};

/* Name to prepend to messages. */
static const char libname[] = "link-grammar";

/* === Error queue utilities ======================================== */
static lg_errinfo *error_queue_resize(lg_errinfo *lge, int len)
{
	lge = realloc(lge, (len+2) * sizeof(lg_errinfo));
	lge[len+1].text = NULL;
	return lge;
}

static int error_queue_len(lg_errinfo *lge)
{
	size_t len = 0;
	if (lge)
		while (NULL != lge[len].text) len++;
	return len;
}

static void error_queue_append(lg_errinfo **lge, lg_errinfo *current_error)
{
	int n = error_queue_len(*lge);

	*lge = error_queue_resize(*lge, n);
	current_error->text = strdup(current_error->text);
	(*lge)[n] = *current_error;
}
/* ==================================================================*/

/**
 * Return the error severity according to the start of the error string.
 * If an error severity is not found - return None.
 */
static lg_error_severity message_error_severity(const char *msgtext)
{
	for (const char **llp = severity_label_by_level; NULL != *llp; llp++)
	{
		for (const char *s = *llp, *t = msgtext; ; s++, t++)
		{
			if ((':' == *t) && (t > msgtext))
			{
				return (int)(llp - severity_label_by_level + 1);
			}
			if ((*s != *t) || ('\0' == *s)) break;
		}
	}

	return lg_None;
}

static void lg_error_msg_free(lg_errinfo *lge)
{
		free((void *)lge->text);
		free((void *)lge->severity_label);
}

/* === API functions ================================================*/
/**
 * Set the error handler function to the given one.
 * @param lg_error_handler New error handler function
 * @param data Argument for the error handler function
 */
lg_error_handler lg_error_set_handler(lg_error_handler f, void *data)
{
	const lg_error_handler oldf = lg_error.handler;
	lg_error.handler = f;
	lg_error.handler_data = data;
	return oldf;
}

const void *lg_error_set_handler_data(void * data)
{
	const char *old_data = lg_error.handler_data;

	lg_error.handler_data = data;
	return old_data;
}

/**
 * Print the error queue and free it.
 * @param f Error handler function
 * @param data Argument for the error handler function
 * @return Number of errors
 */
int lg_error_printall(lg_error_handler f, void *data)
{
	int n = error_queue_len(lg_error.errmsg);
	if (0 == n) return 0;

	for (lg_errinfo *lge = &lg_error.errmsg[n-1]; lge >= lg_error.errmsg; lge--)
	{
		if (NULL == f)
			default_error_handler(lge, data);
		else
			f(lg_error.errmsg, data);
		lg_error_msg_free(lge);
	}
	free(lg_error.errmsg);
	lg_error.errmsg = NULL;

	return n;
}

/**
 * Clear the error queue. Free all of its memory.
 * @return Number of errors
 */
int lg_error_clearall(void)
{
	if (NULL == lg_error.errmsg) return 0;
	int nerrors = 0;

	for (lg_errinfo *lge = lg_error.errmsg; NULL != lge->text; lge++)
	{
		nerrors++;
		lg_error_msg_free(lge);
	}
	free(lg_error.errmsg);
	lg_error.errmsg = NULL;

	return nerrors;
}

/**
 * Format the given raw error message.
 * Create a complete error message, ready to be printed.
 * If the severity is not lg_None, add the library name.
 * Also add the severity label.
 * @param lge The raw error message.
 * @return The complete error message. The caller needs to free the memory.
 */
char *lg_error_formatmsg(lg_errinfo *lge)
{
	dyn_str *s = dyn_str_new();

	/* Prepend libname to messages with higher severity than Debug. */
	if (lge->severity < lg_Debug)
		append_string(s, "%s: ", libname);

	if ((NULL != lge->severity_label) && ('\0' != lge->severity_label[0]))
		append_string(s, "%s: ", lge->severity_label);

	append_string(s, "%s", lge->text);

	return dyn_str_take(s);
}

static TLS dyn_str *outbuf = NULL;

/**
 * Flush a partial error message if exists.
 * Return true iff a message has been actually flushed.
 *
 * (Just using prt_error("\n") also flushes a buffered partial error
 * message, but if there is no such message an empty message is generated).
 */
bool lg_error_flush(void)
{
	if (outbuf == NULL) return false;
	prt_error("\n");
	return true;
}
/* ================================================================== */

/**
 * The default error handler callback function.
 * @param lge The raw error message.
 */
static void default_error_handler(lg_errinfo *lge, void *data)
{
	FILE *outfile = stdout;

	if (((NULL == data) && (lge->severity <= lg_Debug)) ||
	    ((NULL != data) && (lge->severity <= *(lg_error_severity *)data) &&
	     (lg_None !=  lge->severity)))
	if (((NULL == data) && (lge->severity < lg_Debug)) ||
	    ((NULL != data) && (lge->severity < *(lg_error_severity *)data) &&
	     (lg_None !=  lge->severity)))
	{
		fflush(stdout); /* Make sure that stdout has been written out first. */
		outfile = stderr;
	}

	char *msgtext = lg_error_formatmsg(lge);
	fprintf(outfile, "%s", msgtext);
	free(msgtext);

	fflush(outfile); /* Also stderr, in case some OS does some strange thing */
}

/**
 * Convert a numerical severity level to its corresponding string.
 */
static const char *error_severity_label(lg_error_severity sev)
{
	char *sevlabel = alloca(MAX_SEVERITY_LABEL_SIZE);

	if (lg_None == sev)
	{
		sevlabel[0] = '\0';
	}
	else if ((sev < 1) || (sev > lg_None))
	{
		snprintf(sevlabel, MAX_SEVERITY_LABEL_SIZE, "Message severity %d", sev);
	}
	else
	{
		sevlabel = (char *)severity_label_by_level[sev-1];
	}

	return strdup(sevlabel);
}

static void verr_msg(err_ctxt *ec, lg_error_severity sev, const char *fmt, va_list args)
	GNUC_PRINTF(3,0);

static void verr_msg(err_ctxt *ec, lg_error_severity sev, const char *fmt, va_list args)
{
	if (NULL == outbuf) outbuf = dyn_str_new();

	/*
	 * If the message is a complete one, it ends with a newline.  Else the
	 * message is buffered in msg_buf until it is complete. A complete line
	 * which is not a complete message is marked with a \ at its end (after
	 * its newline), which is removed here. The newline and \ should be
	 * specified only in the format string.
	 */
	char *nfmt;
	bool partline = false;
	const int fmtlen = strlen(fmt);

	if ('\n' != fmt[fmtlen-1])
	{
		partline = true;
		if ('\\' == fmt[fmtlen-1])
		{
			nfmt = strdupa(fmt);
			nfmt[fmtlen-1] = '\0';
			fmt = nfmt;
		}
	}
	vappend_string(outbuf, fmt, args);
	if (partline) return;

	if ((NULL != ec) && (NULL != ec->sent))
		print_sentence_context(ec->sent, outbuf);

	lg_errinfo current_error;
	/* current_error.ec = *ec; */
	const char *error_text = outbuf->str;
	lg_error_severity msg_sev = message_error_severity(error_text);
	if (lg_None != msg_sev)
	{
		/* Strip off the error severity label, for consistency.
		 * lg_error_format() will reconstruct it. */
		error_text = strchr(error_text, ':') + 1;
		error_text += strspn(error_text, " \t");
	}
	current_error.text = error_text;
	current_error.severity = ((lg_None == msg_sev) && (0 != sev)) ? sev : msg_sev;
	current_error.severity_label = error_severity_label(current_error.severity);

	if (NULL == lg_error.handler)
	{
		error_queue_append(&lg_error.errmsg, &current_error);
	}
	else
	{
		lg_error.handler(&current_error, lg_error.handler_data);
		free((void *)current_error.severity_label);
	}

	dyn_str_delete(outbuf);
	outbuf = NULL;
}

void err_msgc(err_ctxt *ec, lg_error_severity sev, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	verr_msg(ec, sev, fmt, args);
	va_end(args);
}

/**
 * Issue the given message.
 * This is an API function.
 *
 * Usage notes:
 * The severity can be specified as an initial string in the message,
 * such as "Error: Rest of message".  For known severity names see
 * \link severity_label_by_level List of severity strings. \endlink.
 * See \link verr_msg \endlink for how the severity is handled
 * if it is not specified.
 *
 * @fmt printf()-like format.
 * @... printf()-like arguments.
 * @return Always 0, not to be used. This is needed so prt_error()
 * can be used in complex macros that have to use the comma operator.
 */
int prt_error(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	verr_msg(NULL, 0, fmt, args);
	va_end(args);

	return 0;
}

/**
 * Check whether the given feature is enabled. It is considered
 * enabled if it is found in the comma delimited list of features.
 * This list, if not empty, has a leading and a trailing commas.
 * Return NULL if not enabled, else ",". If the feature appears
 * as "feature:param", return a pointer to param.
 * @param    list Comma delimited list of features (start/end commas too).
 * @param    ... List of features to check.
 * @return   If not enabled - NULL; Else "," or the feature param if exists.
 */
const char *feature_enabled(const char * list, ...)
{

	const char *feature;
	va_list given_features;
	va_start(given_features, list);

	while (NULL != (feature = va_arg(given_features, char *)))
	{
		size_t len = strlen(feature);
		char *buff = alloca(len + 2 + 1); /* leading comma + comma/colon + NUL */

		/* The "feature" variable may contain a full/relative file path.
		 * If so, extract the file name from it. On Windows first try the
		 * native separator \, but also try /. */
		const char *dir_sep = NULL;
#ifdef _WIN32
		dir_sep = strrchr(feature, '\\');
#endif
		if (NULL == dir_sep) dir_sep = strrchr(feature, '/');
		if (NULL != dir_sep) feature = dir_sep + 1;

		buff[0] = ',';
		strcpy(buff+1, feature);
		strcat(buff, ",");

		if (NULL != strstr(list, buff)) return ",";
		buff[len+1] = ':'; /* check for "feature:param" */
		if (NULL != strstr(list, buff)) return strstr(list, buff) + len + 1;
	}
	va_end(given_features);

	return NULL;
}

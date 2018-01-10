/***************************************************************************/
/* Copyright (c) 2012 Linas Vepstas                                        */
/* All rights reserved                                                     */
/*                                                                         */
/* Use of the link grammar parsing system is subject to the terms of the   */
/* license set forth in the LICENSE file included with this software.      */
/* This license allows free redistribution and use in source and binary    */
/* forms, with or without modification, subject to certain conditions.     */
/*                                                                         */
/***************************************************************************/

/**
 * Arghhhh. This hacks around multiple stupidities in readline/editline.
 * 1) most versions of editline don't have wide-char support.
 * 2) No versions of editline have UTF8 support.
 * So basically readline() is just plain broken.
 * So hack one up, using the wide-char interfaces.  This is a hack. Argh.
 *
 * Double-arghh.  Current versions of readline hang in an infinite loop
 * on __read_nocancel() in read_char() called from el_wgets() (line 92
 * below) when the input is "He said 《 This is bull shit 》" Notice
 * the unicode angle-brackets.
 */

#include "lg_readline.h"

#ifdef HAVE_EDITLINE
#include <string.h>
#include <histedit.h>
#include <stdlib.h>

#ifdef HAVE_WIDECHAR_EDITLINE
#include <stdbool.h>

static wchar_t * wc_prompt = NULL;
static wchar_t * prompt(EditLine *el)
{
	return wc_prompt;
}

char *lg_readline(const char *mb_prompt)
{
	static bool is_init = false;
	static HistoryW *hist = NULL;
	static HistEventW ev;
	static EditLine *el = NULL;
	static char *mb_line;

	int numc;
	size_t byte_len;
	const wchar_t *wc_line;
	char *nl;

	if (!is_init)
	{
		size_t sz;
#define HFILE ".lg_history"
		is_init = true;

		sz = mbstowcs(NULL, mb_prompt, 0) + 4;
		wc_prompt = malloc (sz*sizeof(wchar_t));
		mbstowcs(wc_prompt, mb_prompt, sz);

		hist = history_winit();    /* Init built-in history */
		el = el_init("link-parser", stdin, stdout, stderr);
		history_w(hist, &ev, H_SETSIZE, 20);   /* Remember 20 events */
		el_wset(el, EL_HIST, history_w, hist);
		el_source(el, NULL);       /* Source the user's defaults file. */
		history_w(hist, &ev, H_LOAD, HFILE);

		el_set(el, EL_SIGNAL, 1); /* Restore tty setting on returning to shell */

		/* By default, it comes up in vi mode, with the editor not in
		 * insert mode; and even when in insert mode, it drops back to
		 * command mode at the drop of a hat. Totally confusing/lame. */
		el_wset(el, EL_EDITOR, L"emacs");
		el_wset(el, EL_PROMPT_ESC, prompt, '\1'); /* Set the prompt function */
	}

	wc_line = el_wgets(el, &numc);

	/* Received end-of-file */
	if (numc <= 0)
	{
		el_end(el);
		history_wend(hist);
		free(wc_prompt);
		wc_prompt = NULL;
		hist = NULL;
		el = NULL;
		is_init = false;
		return NULL;
	}

	if (1 < numc)
	{
		history_w(hist, &ev, H_ENTER, wc_line);
		history_w(hist, &ev, H_SAVE, HFILE);
	}
	/* fwprintf(stderr, L"==> got %d %ls", numc, wc_line); */

	byte_len = wcstombs(NULL, wc_line, 0) + 4;
	free(mb_line);
	mb_line = malloc(byte_len);
	wcstombs(mb_line, wc_line, byte_len);

	/* In order to be compatible with regular libedit, we have to
	 * strip away the trailing newline, if any. */
	nl = strchr(mb_line, '\n');
	if (nl) *nl = 0x0;

	return mb_line;
}

#else /* HAVE_WIDECHAR_EDITLINE */

#include <editline/readline.h>

char *lg_readline(const char *prompt)
{
	static char *pline;

	free(pline);
	pline = readline(prompt);

	/* Save non-blank lines */
	if (pline && *pline)
	{
		if (*pline) add_history(pline);
	}

	return pline;
}
#endif /* HAVE_WIDECHAR_EDITLINE */
#endif /* HAVE_EDITLINE */

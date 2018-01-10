/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2014 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <link-grammar/link-features.h>
#include <link-grammar/link-includes.h>

LINK_BEGIN_DECLS  /* Needed to keep MSVC6 happy */

typedef struct {
	Parse_Options popts;
	Parse_Options panic_opts;

	size_t screen_width;    /* width of screen for displaying linkages */
	bool batch_mode;        /* if true, process sentences non-interactively */
	bool allow_null;        /* true if we allow null links in parsing */
	bool echo_on;           /* true if we should echo the input sentence */
	bool panic_mode;        /* if true, parse in "panic mode" after all else fails */
	bool display_on;        /* if true, output graphical linkage diagram */
	bool display_walls;     /* if true, show the wall words in the linkage diagram */
	bool display_postscript;/* if true, output postscript linkage */
	bool display_ps_header; /* if true, output postscript headers */
	ConstituentDisplayStyle display_constituents; /* style for displaying constituent structure */

	bool display_bad;       /* if true, bad linkages are displayed */
	bool display_disjuncts; /* if true, print disjuncts that were used */
	bool display_links;     /* if true, a list o' links is printed out */
	bool display_senses;    /* if true, sense candidates are printed out */
} Command_Options;

LINK_END_DECLS

int issue_special_command(const char*, Command_Options*, Dictionary);
Command_Options* command_options_create(void);
void command_options_delete(Command_Options*);



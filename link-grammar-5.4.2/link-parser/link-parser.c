/***************************************************************************/
/* Copyright (c) 2004                                                      */
/* Daniel Sleator, David Temperley, and John Lafferty                      */
/* Copyright (c) 2008, 2014 Linas Vepstas                                  */
/* All rights reserved                                                     */
/*                                                                         */
/* Use of the link grammar parsing system is subject to the terms of the   */
/* license set forth in the LICENSE file included with this software.      */
/* This license allows free redistribution and use in source and binary    */
/* forms, with or without modification, subject to certain conditions.     */
/*                                                                         */
/***************************************************************************/

 /****************************************************************************
 *
 *   This is a simple example of the link parser API.  It simulates most of
 *   the functionality of the original link grammar parser, allowing sentences
 *   to be typed in either interactively or in "batch" mode (if -batch is
 *   specified on the command line, and stdin is redirected to a file).
 *   The program:
 *     Opens up a dictionary
 *     Iterates:
 *        1. Reads from stdin to get an input string to parse
 *        2. Tokenizes the string to form a Sentence
 *        3. Tries to parse it with cost 0
 *        4. Tries to parse with increasing cost
 *     When a parse is found:
 *        1. Extracts each Linkage
 *        2. Passes it to process_some_linkages()
 *        3. Deletes linkage
 *     After parsing each Sentence is deleted by making a call to
 *     sentence_delete.
 *
 ****************************************************************************/

#include <errno.h>
#include <locale.h>
#include <stdlib.h>
#include <string.h>

/* Used for terminal resizing */
#ifndef _WIN32
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#else
#include <windows.h>
#include <wchar.h>
#include <io.h>
#endif /* _WIN32 */

#ifdef _MSC_VER
#define LINK_GRAMMAR_DLL_EXPORT 0
#endif /* _MSC_VER */

#ifndef _WIN32
#define LAST_RESORT_LOCALE "en_US.UTF-8" /* Supposing POSIX systems */
#else
#define LAST_RESORT_LOCALE ""            /* Use user default locale */
#endif /* _WIN32 */

#include "parser-utilities.h"
#include "command-line.h"
#include "lg_readline.h"
#ifdef USE_VITERBI
#include "../viterbi/viterbi.h"
#endif /* USE_VITERBI */

#define DISPLAY_MAX 1024
#define COMMENT_CHAR '%'  /* input lines beginning with this are ignored */
#define WHITESPACE " \t\v\r\n" /* ASCII-only is sufficient here. */

static int batch_errors = 0;
static int verbosity = 0;
static char * debug = (char *)"";
static char * test = (char *)"";
static bool isatty_stdin, isatty_stdout;
#ifdef _WIN32
static bool running_under_cygwin = false;
#endif /* _WIN32 */

typedef enum
{
	UNGRAMMATICAL = '*',
	PARSE_WITH_DISJUNCT_COST_GT_0 = ':',  /* Not used anywhere, currently ... */
	NO_LABEL = ' '
} Label;

static char * get_terminal_line(char *input_string, FILE *in, FILE *out)
{
	static char *pline;
	const char *prompt = (0 == verbosity)? "" : "linkparser> ";

#ifdef HAVE_EDITLINE
	#ifdef _WIN32
		#error __FILE__ ": Cannot use HAVE_EDITLINE "
		                "(the console already has line editing and history)."
	#endif /* _WIN32 */
	pline = lg_readline(prompt);
#else
	fprintf(out, "%s", prompt);
	fflush(out);
#ifdef _WIN32
	if (!running_under_cygwin)
		pline = get_console_line();
	else
		pline = fgets(input_string, MAX_INPUT, in);
#else
	pline = fgets(input_string, MAX_INPUT, in);
#endif /* _WIN32 */
#endif /* HAVE_EDITLINE */

	return pline;
}

static char * fget_input_string(FILE *in, FILE *out, bool check_return)
{
	static char *pline;
	static char input_string[MAX_INPUT];
	static bool input_pending = false;

	if (input_pending)
	{
		input_pending = false;
		return pline;
	}

	input_string[MAX_INPUT-2] = '\0';

	if ((in != stdin) || !isatty_stdin)
	{
		/* Get input from a file. */
		pline = fgets(input_string, MAX_INPUT, in);
	}
	else
	{
		/* If we are here, the input is from a terminal. */
		pline = get_terminal_line(input_string, in, out);
	}

	if (NULL == pline) return NULL;      /* EOF */

	if (('\0' != input_string[MAX_INPUT-2]) &&
	    ('\n' != input_string[MAX_INPUT-2]))
	{
		prt_error("Warning: Input line too long (>%d)\n", MAX_INPUT-1);
		/* TODO: Ignore it and its continuation part(s). */
	}

	if (check_return)
	{
		if (('\0' == pline[0]) || ('\r' == pline[0]) || ('\n' == pline[0]))
			return (char *)"\n";           /* Continue linkage display */
		input_pending = true;
		return (char *)"x";               /* Stop linkage display */
	}

	return pline;
}

/**************************************************************************
*
*  This procedure displays a linkage graphically.  Since the diagrams
*  are passed as character strings, they need to be deleted with a
*  call to free.
*
**************************************************************************/

static void process_linkage(Linkage linkage, Command_Options* copts)
{
	char * string;
	ConstituentDisplayStyle mode;

	if (!linkage) return;  /* Can happen in timeout mode */

	if (copts->display_bad)
	{
		string = linkage_print_pp_msgs(linkage);
		fprintf(stdout, "%s\n", string);
		linkage_free_pp_msgs(string);
	}
	if (copts->display_on)
	{
		string = linkage_print_diagram(linkage, copts->display_walls, copts->screen_width);
		fprintf(stdout, "%s", string);
		linkage_free_diagram(string);
	}
	if ((mode = copts->display_constituents))
	{
		string = linkage_print_constituent_tree(linkage, mode);
		if (string != NULL)
		{
			fprintf(stdout, "%s\n", string);
			linkage_free_constituent_tree_str(string);
		}
		else
		{
			copts->display_constituents = 0;
			fprintf(stderr, "Can't generate constituents.\n");
			fprintf(stderr, "Constituent processing has been turned off.\n");
		}
	}
	if (copts->display_links)
	{
		string = linkage_print_links_and_domains(linkage);
		fprintf(stdout, "%s", string);
		linkage_free_links_and_domains(string);
	}
	if (copts->display_senses)
	{
		string = linkage_print_senses(linkage);
		fprintf(stdout, "%s", string);
		linkage_free_senses(string);
	}
	if (copts->display_disjuncts)
	{
		string = linkage_print_disjuncts(linkage);
		fprintf(stdout, "%s\n", string);
		linkage_free_disjuncts(string);
	}
	if (copts->display_postscript)
	{
		string = linkage_print_postscript(linkage,
		          copts->display_walls, copts->display_ps_header);
		fprintf(stdout, "%s\n", string);
		linkage_free_postscript(string);
	}
}

static void print_parse_statistics(Sentence sent, Parse_Options opts)
{
	if (sentence_num_linkages_found(sent) > 0)
	{
		if (sentence_num_linkages_found(sent) >
			parse_options_get_linkage_limit(opts))
		{
			fprintf(stdout, "Found %d linkage%s (%d of %d random " \
					"linkages had no P.P. violations)",
					sentence_num_linkages_found(sent),
					sentence_num_linkages_found(sent) == 1 ? "" : "s",
					sentence_num_valid_linkages(sent),
					sentence_num_linkages_post_processed(sent));
		}
		else
		{
			fprintf(stdout, "Found %d linkage%s (%d had no P.P. violations)",
					sentence_num_linkages_post_processed(sent),
					sentence_num_linkages_post_processed(sent) == 1 ? "" : "s",
					sentence_num_valid_linkages(sent));
		}
		if (sentence_null_count(sent) > 0)
		{
			fprintf(stdout, " at null count %d", sentence_null_count(sent));
		}
		fprintf(stdout, "\n");
	}
}

/**
 * Check for the auto-next-linkage test request (for LG code development).
 * It is given using the special command: test=auto-next-linkage[:display_max]
 * when :display_max is an optional indication of the maximum number of
 * linkages to auto-display (the default is DISPLAY_MAX).
 * For example, to issue up to 20000 linkages for each batch sentence,
 * the following can be used:
 * link-parser -limit=30000 -test=auto-next-linkage:20000 < file.batch
 */
static int auto_next_linkage_test(const char *test_opt)
{
	char auto_next_linkage_str[] = ",auto-next-linkage";
	char *auto_next_linkage_pos = strstr(test_opt, auto_next_linkage_str);
	int max_display;

	if (auto_next_linkage_pos == NULL) return 0;
	max_display = atoi(auto_next_linkage_pos + sizeof(auto_next_linkage_str));
	if (max_display != 0) return max_display;
	return DISPLAY_MAX;
}

static const char *process_some_linkages(FILE *in, Sentence sent,
                                         Command_Options* copts)
{
	int i, num_to_query, num_to_display, num_displayed;
	Linkage linkage;
	double corpus_cost;
	Parse_Options opts = copts->popts;
	int display_max = DISPLAY_MAX;
	bool auto_next_linkage = false;

	i = auto_next_linkage_test(test);
	if (i != 0)
	{
		display_max = i;
		auto_next_linkage = true;
	}

	if (verbosity > 0) print_parse_statistics(sent, opts);
	num_to_query = sentence_num_linkages_post_processed(sent);
	if (!copts->display_bad)
	{
		num_to_display = MIN(sentence_num_valid_linkages(sent),
		                     display_max);
	}
	else
	{
		num_to_display = MIN(num_to_query, display_max);
	}

	for (i=0, num_displayed=0; i<num_to_query; i++)
	{
		if ((sentence_num_violations(sent, i) > 0) &&
			!copts->display_bad)
		{
			continue;
		}

		linkage = linkage_create(i, sent, opts);

		/* Currently, sat solver sets the linkage violation indication
		 * only when it creates the linkage as a result of the above call. */
		if ((sentence_num_violations(sent, i) > 0) &&
			!copts->display_bad)
		{
			continue;
		}

		/* Currently, sat solver returns NULL when there ain't no more */
		if (!linkage)
		{
			if (verbosity > 0)
			{
				if (0 == i)
					fprintf(stdout, "No linkages found.\n");
				else
					fprintf(stdout, "No more linkages.\n");
			}
			break;
		}

		if (verbosity > 0)
		{
			if ((sentence_num_valid_linkages(sent) == 1) &&
				!copts->display_bad)
			{
				fprintf(stdout, "	Unique linkage, ");
			}
			else if (copts->display_bad &&
			         (sentence_num_violations(sent, i) > 0))
			{
				fprintf(stdout, "	Linkage %d (bad), ", num_displayed+1);
			}
			else
			{
				fprintf(stdout, "	Linkage %d, ", num_displayed+1);
			}

			corpus_cost = linkage_corpus_cost(linkage);
			if (corpus_cost < 0.0f)
			{
				fprintf(stdout, "cost vector = (UNUSED=%d DIS=%5.2f LEN=%d)\n",
				       linkage_unused_word_cost(linkage),
				       linkage_disjunct_cost(linkage),
				       linkage_link_cost(linkage));
			}
			else
			{
				fprintf(stdout, "cost vector = (CORP=%6.4f UNUSED=%d DIS=%5.2f LEN=%d)\n",
				       corpus_cost,
				       linkage_unused_word_cost(linkage),
				       linkage_disjunct_cost(linkage),
				       linkage_link_cost(linkage));
			}
		}

		process_linkage(linkage, copts);
		linkage_delete(linkage);

		if (++num_displayed < num_to_display)
		{
			if (!auto_next_linkage)
			{
				if ((verbosity > 0) && (in == stdin) && isatty_stdin && isatty_stdout)
				{
					fprintf(stdout, "Press RETURN for the next linkage.\n");
				}
				char *rc = fget_input_string(in, stdout, /*check_return*/true);
				if ((NULL == rc) || (*rc != '\n')) return rc;
			}
		}
		else
		{
			break;
		}
	}
	return "x";
}

static int there_was_an_error(Label label, Sentence sent, Parse_Options opts)
{
	if (sentence_num_valid_linkages(sent) > 0) {
		if (label == UNGRAMMATICAL) {
			batch_errors++;
			return UNGRAMMATICAL;
		}
		if ((sentence_disjunct_cost(sent, 0) == 0.0) &&
			(label == PARSE_WITH_DISJUNCT_COST_GT_0)) {
			batch_errors++;
			return PARSE_WITH_DISJUNCT_COST_GT_0;
		}
	} else {
		if (label != UNGRAMMATICAL) {
			batch_errors++;
			return UNGRAMMATICAL;
		}
	}
	return 0;
}

static void batch_process_some_linkages(Label label,
                                        Sentence sent,
                                        Command_Options* copts)
{
	Parse_Options opts = copts->popts;

	if (there_was_an_error(label, sent, opts))
	{
		/* If we found at least one good linkage, print it. */
		if (sentence_num_valid_linkages(sent) > 0) {
			Linkage linkage = NULL;
			int i;
			for (i=0; i<sentence_num_linkages_post_processed(sent); i++)
			{
				if (0 == sentence_num_violations(sent, i))
				{
					linkage = linkage_create(i, sent, opts);
					break;
				}
			}
			process_linkage(linkage, copts);
			linkage_delete(linkage);
		}
		fprintf(stdout, "+++++ error %d\n", batch_errors);
	}
	else
	{
		if (strstr(test, ",batch_print_parse_statistics,"))
		{
			print_parse_statistics(sent, opts);
		}
	}
}

static bool special_command(char *input_string, Command_Options* copts, Dictionary dict)
{
	if (input_string[0] == COMMENT_CHAR) return true;
	if (input_string[0] == '!') {
		issue_special_command(input_string+1, copts, dict);
		return true;
	}
	return false;
}

static Label strip_off_label(char * input_string)
{
	Label c;

	c = (Label) input_string[0];
	switch(c) {
	case UNGRAMMATICAL:
	case PARSE_WITH_DISJUNCT_COST_GT_0:
		input_string[0] = ' ';
		return c;
	case NO_LABEL:
	default:
		return NO_LABEL;
	}
}

static void setup_panic_parse_options(Parse_Options opts)
{
	parse_options_set_disjunct_cost(opts, 4.0f);
	parse_options_set_min_null_count(opts, 1);
	parse_options_set_max_null_count(opts, 100);
	parse_options_set_max_parse_time(opts, 60);
	parse_options_set_islands_ok(opts, false);
	parse_options_set_short_length(opts, 12);
	parse_options_set_all_short_connectors(opts, 1);
	parse_options_set_linkage_limit(opts, 100);
	parse_options_set_spell_guess(opts, 0);
}

static void print_usage(char *str)
{
	Command_Options *copts;
	fprintf(stderr,
			"Usage: %s [language|dictionary location]\n"
			"                   [-<special \"!\" command>]\n"
			"                   [--version]\n", str);

	fprintf(stderr, "\nSpecial commands are:\n");
	copts = command_options_create();
	issue_special_command("var", copts, NULL);
	exit(-1);
}

/**
 * On Unix, this checks for the current window size,
 * and sets the output screen width accordingly.
 * Not sure how MS Windows does this.
 */
static void check_winsize(Command_Options* copts)
{
	if (!isatty_stdout) return;
	int fd = fileno(stdout);
#ifdef _WIN32
	HANDLE console;
	CONSOLE_SCREEN_BUFFER_INFO info;

	/* Create a handle to the console screen. */
	console = (HANDLE)_get_osfhandle(fd);
	if (!console || (console == INVALID_HANDLE_VALUE)) goto fail;

	/* Calculate the size of the console window. */
	if (GetConsoleScreenBufferInfo(console, &info) == 0) goto fail;

	copts->screen_width = info.srWindow.Right - info.srWindow.Left;
	return;

fail:
	copts->screen_width = 79;
	return;
#else
	struct winsize ws;

	/* If there is no controlling terminal, the fileno will fail. This
	 * seems to happen while building docker images, I don't know why.
	 */
	if (fd < 0) return;

	if (0 != ioctl(fd, TIOCGWINSZ, &ws))
	{
		perror("stdout: ioctl TIOCGWINSZ");
		return;
	}

	/* printf("rows %i\n", ws.ws_row); */
	/* printf("cols %i\n", ws.ws_col); */

	/* Set the screen width only if the returned value seems
	 * rational: its positive and not insanely tiny.
	 */
	if ((10 < ws.ws_col) && (16123 > ws.ws_col))
	{
		copts->screen_width = ws.ws_col - 1;
	}
#endif /* _WIN32 */
}

int main(int argc, char * argv[])
{
	FILE            *input_fh = stdin;
	Dictionary      dict;
	const char     *language = NULL;
	int             num_linkages, i;
	Label           label = NO_LABEL;
	Command_Options *copts;
	Parse_Options   opts;
	bool batch_in_progress = false;

	isatty_stdin = isatty(fileno(stdin));
	isatty_stdout = isatty(fileno(stdout));

#ifdef _WIN32
	/* If compiled with MSVC/MSYS, we still support running under Cygwin.
	 * This is done by checking running_under_cygwin to resolve
	 * incompatibilities. */
	const char *ostype = getenv("OSTYPE");
	if ((NULL != ostype) && (0 == strcmp(ostype, "cygwin")))
		running_under_cygwin = true;
#endif /* _WIN32 */

#if LATER
	/* Try to catch the SIGWINCH ... except this is not working. */
	struct sigaction winch_act;
	winch_act.sa_handler = winch_handler;
	winch_act.sa_sigaction = NULL;
	sigemptyset (&winch_act.sa_mask);
	winch_act.sa_flags = 0;
	sigaction (SIGWINCH, &winch_act, NULL);
#endif

	i = 1;
	if ((argc > 1) && (argv[1][0] != '-')) {
		/* the dictionary is the first argument if it doesn't begin with "-" */
		language = argv[1];
		i++;
	}

	for (; i<argc; i++)
	{
		if (argv[i][0] == '-' && strcmp("--version", argv[i]) == 0)
		{
			printf("Version: %s\n", linkgrammar_get_version());
			exit(0);
		}
	}

	copts = command_options_create();
	if (copts == NULL || copts->panic_opts == NULL)
	{
		prt_error("Fatal error: unable to create parse options\n");
		exit(-1);
	}
	opts = copts->popts;

	setup_panic_parse_options(copts->panic_opts);
	copts->panic_mode = true;

	parse_options_set_max_parse_time(opts, 30);
	parse_options_set_linkage_limit(opts, 1000);
	parse_options_set_min_null_count(opts, 0);
	parse_options_set_max_null_count(opts, 0);
	parse_options_set_short_length(opts, 16);
	parse_options_set_islands_ok(opts, false);

	/* Process command line variable-setting commands (only) */
	for (i = 1; i < argc; i++)
	{
		if (argv[i][0] == '-')
		{
			const char *var = argv[i] + ((argv[i][1] != '-') ? 1 : 2);
			if ((var[0] != '!') && issue_special_command(var, copts, NULL))
				print_usage(argv[0]);
		}
	}

#ifdef _WIN32
	win32_set_utf8_output();
#endif /* _WIN32 */

	if (language && *language)
	{
		dict = dictionary_create_lang(language);
		if (dict == NULL)
		{
			prt_error("Fatal error: Unable to open dictionary.\n");
			exit(-1);
		}
	}
	else
	{
		dict = dictionary_create_default_lang();
		if (dict == NULL)
		{
			prt_error("Fatal error: Unable to open default dictionary.\n");
			exit(-1);
		}
	}

	/* Process the command line '!' commands */
	for (i = 1; i<argc; i++)
	{
		if ((argv[i][0] == '-') && (argv[i][1] == '!'))
		{
			if (issue_special_command(argv[i]+1, copts, dict))
				print_usage(argv[0]);
		}
	}

	check_winsize(copts);

	prt_error("Info: Dictionary version %s, locale %s\n",
		linkgrammar_get_dict_version(dict),
		linkgrammar_get_dict_locale(dict));
	prt_error("Info: Library version %s. Enter \"!help\" for help.\n",
		linkgrammar_get_version());

	/* Main input loop */
	while (true)
	{
		char *input_string;
		Sentence sent = NULL;

		verbosity = parse_options_get_verbosity(opts);
		debug = parse_options_get_debug(opts);
		test = parse_options_get_test(opts);

		input_string = fget_input_string(input_fh, stdout, /*check_return*/false);
		check_winsize(copts);

		if (NULL == input_string)
		{
			if (input_fh == stdin) break;
			fclose (input_fh);
			input_fh = stdin;
			continue;
		}

		/* Discard whitespace characters from end of string. */
		for (char *p = &input_string[strlen(input_string)-1];
		     (p > input_string) && strchr(WHITESPACE, *p) ; p--)
		{
			*p = '\0';
		}

		if ((strcmp(input_string, "!quit") == 0) ||
		    (strcmp(input_string, "!exit") == 0)) break;

		/* We have to handle the !file command inline; its too hairy
		 * otherwise ... */
		if (strncmp(input_string, "!file", 5) == 0)
		{
			char * filename = &input_string[6];
			int fnlen = strlen(filename);

			if ('\n' == filename[fnlen-1]) filename[fnlen-1] = '\0';

			input_fh = fopen(filename, "r");
			if (NULL == input_fh)
			{
				int perr = errno;
				fprintf(stderr, "Error: %s (%d) %s\n",
				        filename, perr, strerror(perr));
				input_fh = stdin;
				continue;
			}
			continue;
		}

		/* If the input string is just whitespace, then ignore it. */
		if (strspn(input_string, WHITESPACE) == strlen(input_string))
			continue;

		if (special_command(input_string, copts, dict)) continue;

		if (!copts->batch_mode) batch_in_progress = false;
		if ('\0' != test[0])
		{
			/* In batch mode warn only once.
			 * In auto-next-linkage mode don't warn at all. */
			if (!batch_in_progress && !auto_next_linkage_test(test))
			{
				fflush(stdout);
				/* Remind the developer this is a test mode. */
				fprintf(stderr, "Warning: Tests enabled: %s\n", test);
				if (copts->batch_mode) batch_in_progress = true;
			}
		}

		if (copts->echo_on)
		{
			printf("%s\n", input_string);
		}

		if (copts->batch_mode)
		{
			label = strip_off_label(input_string);
		}

		// Post-processing-based pruning will clip away connectors
		// that we might otherwise want to examine. So disable PP
		// pruning in this situation.
		if (copts->display_bad)
			parse_options_set_perform_pp_prune(opts, false);
		else
			parse_options_set_perform_pp_prune(opts, true);

#ifdef USE_VITERBI
		/* Compile-time optional, for now, since it don't work yet. */
		if (parse_options_get_use_viterbi(opts))
		{
			viterbi_parse(input_string, dict);
		}
		else
#endif /* USE_VITERBI */
		{
			sent = sentence_create(input_string, dict);

			/* First parse with cost 0 or 1 and no null links */
			// parse_options_set_disjunct_cost(opts, 2.7);
			parse_options_set_min_null_count(opts, 0);
			parse_options_set_max_null_count(opts, 0);
			parse_options_reset_resources(opts);

			num_linkages = sentence_parse(sent, opts);

			/* num_linkages is negative only on a hard-error;
			 * typically, due to a zero-length sentence.  */
			if (num_linkages < 0)
			{
				sentence_delete(sent);
				sent = NULL;
				continue;
			}
#if 0
			/* Try again, this time omitting the requirement for
			 * definite articles, etc. This should allow for the parsing
			 * of newspaper headlines and other clipped speech.
			 *
			 * XXX Unfortunately, this also allows for the parsing of
			 * all sorts of ungrammatical sentences which should not
			 * parse, and leads to bad parses of many other unparsable
			 * but otherwise grammatical sentences.  Thus, this trick
			 * pretty much fails; we leave it here to document the
			 * experiment.
			 */
			if (num_linkages == 0)
			{
				parse_options_set_disjunct_cost(opts, 4.5);
				num_linkages = sentence_parse(sent, opts);
				if (num_linkages < 0) continue;
			}
#endif /* 0 */

			/* Try using a larger list of disjuncts */
			/* XXX FIXME: the lg_expand_disjunct_list() routine is not
			 * currently a part of the public API; it should be made so,
			 * or this expansion idea should be abandoned... not sure which.
			 */
			if ((num_linkages == 0) && parse_options_get_use_cluster_disjuncts(opts))
			{
				int expanded;
				if (verbosity > 0) fprintf(stdout, "No standard linkages, expanding disjunct set.\n");
				parse_options_set_disjunct_cost(opts, 3.9);
				expanded = lg_expand_disjunct_list(sent);
				if (expanded)
				{
					num_linkages = sentence_parse(sent, opts);
				}
				if (0 < num_linkages) printf("Got One !!!!!!!!!!!!!!!!!\n");
			}

			/* If asked to show bad linkages, then show them. */
			if ((num_linkages == 0) && (!copts->batch_mode))
			{
				if (copts->display_bad)
				{
					num_linkages = sentence_num_linkages_found(sent);
				}
			}

			/* Now parse with null links */
			if (num_linkages == 0 && !copts->batch_mode)
			{
				if (verbosity > 0) fprintf(stdout, "No complete linkages found.\n");

				if (copts->allow_null)
				{
					/* XXX should use expanded disjunct list here too */
					parse_options_set_min_null_count(opts, 1);
					parse_options_set_max_null_count(opts, sentence_length(sent));
					num_linkages = sentence_parse(sent, opts);
				}
			}

			if (verbosity > 0)
			{
				if (parse_options_timer_expired(opts))
					fprintf(stdout, "Timer is expired!\n");

				if (parse_options_memory_exhausted(opts))
					fprintf(stdout, "Memory is exhausted!\n");
			}

			if ((num_linkages == 0) &&
				copts->panic_mode &&
				parse_options_resources_exhausted(opts))
			{
				/* print_total_time(opts); */
				batch_errors++;
				if (verbosity > 0) fprintf(stdout, "Entering \"panic\" mode...\n");
				/* If the parser used was the SAT solver, set the panic parser to
				 * it too.
				 * FIXME? Currently, the SAT solver code is not too useful in
				 * panic mode since it doesn't handle parsing with null words, so
				 * using the regular parser in that case could be beneficial.
				 * However, this currently causes a crash due to a memory
				 * management mess. */
				parse_options_set_use_sat_parser(copts->panic_opts,
					parse_options_get_use_sat_parser(opts));
				parse_options_reset_resources(copts->panic_opts);
				parse_options_set_verbosity(copts->panic_opts, verbosity);
				num_linkages = sentence_parse(sent, copts->panic_opts);
				if (verbosity > 0)
				{
					if (parse_options_timer_expired(copts->panic_opts))
						fprintf(stdout, "Panic timer is expired!\n");
				}
			}

			/* print_total_time(opts); */

			if (copts->batch_mode)
			{
				batch_process_some_linkages(label, sent, copts);
			}
			else
			{
				const char *rc = process_some_linkages(input_fh, sent, copts);
				if (NULL == rc)
				{
					sentence_delete(sent);
					sent = NULL;
					break;
				}
			}
			fflush(stdout);

			sentence_delete(sent);
			sent = NULL;
		}
	}

	if (copts->batch_mode)
	{
		/* print_time(opts, "Total"); */
		fprintf(stderr,
				"%d error%s.\n", batch_errors, (batch_errors==1) ? "" : "s");
	}

	/* Free stuff, so that mem-leak detectors don't complain. */
	command_options_delete(copts);
	dictionary_delete(dict);

	printf ("Bye.\n");
	return 0;
}

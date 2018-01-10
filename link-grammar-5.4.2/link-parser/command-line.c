/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2008, 2009, 2011, 2014 Linas Vepstas                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <wctype.h>

#include "parser-utilities.h"
#include "command-line.h"
#include <link-grammar/link-includes.h>

static struct
{
	int verbosity;
	char * debug;
	char * test;
	int timeout;
	int memory;
	int linkage_limit;
	int islands_ok;
	int repeatable_rand;
	int spell_guess;
	int short_length;
	int batch_mode;
	int panic_mode;
	int allow_null;
	int use_cluster_disjuncts;
	int use_sat_solver;
	int use_viterbi;
	int echo_on;
	Cost_Model_type cost_model;
	double max_cost;
	int screen_width;
	int display_on;
	ConstituentDisplayStyle display_constituents;
	int display_postscript;
	int display_ps_header;
	int display_bad;
	int display_links;
	int display_walls;
	int display_disjuncts;
	int display_senses;
	int display_morphology;
} local;

typedef enum
{
	Int,
	Bool,
	Float,
	String,
} ParamType;

typedef struct
{
	const char *string;
	ParamType param_type;
	const char *description;
	void *ptr;
} Switch;

static Switch default_switches[] =
{
	{"bad",        Bool, "Display of bad linkages",         &local.display_bad},
	{"batch",      Bool, "Batch mode",                      &local.batch_mode},
	{"cluster",    Bool, "Use clusters to loosen parsing",  &local.use_cluster_disjuncts},
	{"constituents", Int,  "Generate constituent output",   &local.display_constituents},
	{"cost-model", Int,  "Cost model used for ranking",     &local.cost_model},
	{"cost-max",   Float, "Largest cost to be considered",  &local.max_cost},
	{"disjuncts",  Bool, "Display of disjuncts used",       &local.display_disjuncts},
	{"echo",       Bool, "Echoing of input sentence",       &local.echo_on},
	{"graphics",   Bool, "Graphical display of linkage",    &local.display_on},
	{"islands-ok", Bool, "Use of null-linked islands",      &local.islands_ok},
	{"limit",      Int,  "The maximum linkages processed",  &local.linkage_limit},
	{"links",      Bool, "Display of complete link data",   &local.display_links},
	{"memory",     Int,  "Max memory allowed",              &local.memory},
	{"morphology", Bool, "Display word morphology",         &local.display_morphology},
	{"null",       Bool, "Allow null links",                &local.allow_null},
	{"panic",      Bool, "Use of \"panic mode\"",           &local.panic_mode},
	{"postscript", Bool, "Generate postscript output",      &local.display_postscript},
	{"ps-header",  Bool, "Generate postscript header",      &local.display_ps_header},
	{"rand",       Bool, "Use repeatable random numbers",   &local.repeatable_rand},
	{"senses",     Bool, "Display of word senses",          &local.display_senses},
	{"short",      Int,  "Max length of short links",       &local.short_length},
#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
	{"spell",      Int, "Up to this many spell-guesses per unknown word", &local.spell_guess},
#endif /* HAVE_HUNSPELL */
	{"timeout",    Int,  "Abort parsing after this many seconds", &local.timeout},
#ifdef USE_SAT_SOLVER
	{"use-sat",    Bool, "Use Boolean SAT-based parser",    &local.use_sat_solver},
#endif /* USE_SAT_SOLVER */
	{"verbosity",  Int,  "Level of detail in output",       &local.verbosity},
	{"debug",      String, "comma-separated function list to debug", &local.debug},
	{"test",       String, "comma-separated features to test", &local.test},
#ifdef USE_VITERBI
	{"viterbi",    Bool, "Use Viterbi-based parser",        &local.use_viterbi},
#endif
	{"walls",      Bool, "Display wall words",              &local.display_walls},
	{"width",      Int,  "The width of the display",        &local.screen_width},
	{NULL,         Bool,  NULL,                             NULL}
};

struct {const char * s; const char * str;} user_command[] =
{
	{"variables",	"List user-settable variables and their functions"},
	{"help",	      "List the commands and what they do"},
	{"file",       "Read input from the specified filename"},
	{NULL,		   NULL}
};

/**
 *  Gets rid of all the white space in the string s.  Changes s
 */
static void clean_up_string(char * s)
{
	char * x, * y;
	wchar_t p;
	size_t len, w;
	mbstate_t state;
	memset (&state, 0, sizeof(mbstate_t));

	len = strlen(s);
	y = x = s;
	while(*x != '\0')
	{
		w = mbrtowc(&p, x, len, &state);
		if ((0 == w) || ((size_t)-1 == w)) break;
		len -= w;

		if (!iswspace(p)) {
			while(w) { *y = *x; x++; y++; w--; }
		} else {
			x += w;
		}
	}
	*y = '\0';
}

/**
 * Prints string `s`, aligned to the left, in a field width `w`.
 * If the width of `s` is shorter than `w`, then the remainder of
 * field is padded with blanks (on the right).
 */
static void left_print_string(FILE * fp, const char * s, int w)
{
	int width = w + strlen(s) - utf8_strwidth(s);
	fprintf(fp, "%-*s", width, s);
}

/**
 * Return TRUE if s points to a number:
 * optional + or - followed by 1 or more
 *	digits.
 */
static bool is_numerical_rhs(char *s)
{
	wchar_t p;
	size_t len, w;
	mbstate_t state;
	memset (&state, 0, sizeof(mbstate_t));

	len = strlen(s);

	if (*s=='+' || *s == '-') s++;
	if (*s == '\0') return false;

	for (; *s != '\0'; s+=w)
	{
		w = mbrtowc(&p, s, len, &state);
		if (0 == w) break;
		if ((size_t)-1 == w) return false;
		len -= w;
		if (!iswdigit(p)) return false;
	}
	return true;
}

static int ival(Switch s)
{
	return *((int *) s.ptr);
}

static void setival(Switch s, int val)
{
	*((int *) s.ptr) = val;
}

static int x_issue_special_command(char * line, Command_Options *copts, Dictionary dict)
{
	char *s, *x, *y;
	int i, count, j, k;
	Switch * as = default_switches;
	Parse_Options opts = copts->popts;

	clean_up_string(line);
	s = line;
	j = k = -1;
	count = 0;

	/* Look for boolean flippers */
	for (i=0; as[i].string != NULL; i++)
	{
		if ((Bool == as[i].param_type) &&
		    strncasecmp(s, as[i].string, strlen(s)) == 0)
		{
			count++;
			j = i;
		}
	}

	/* Look for abbreviations */
	for (i=0; user_command[i].s != NULL; i++)
	{
		if (strncasecmp(s, user_command[i].s, strlen(s)) == 0)
		{
			count++;
			k = i;
		}
	}

	if (count > 1)
	{
		printf("Ambiguous command.  Type \"!help\" or \"!variables\"\n");
		return -1;
	}
	else if (count == 1)
	{
		/* flip boolean value */
		if (j >= 0)
		{
			setival(as[j], (0 == ival(as[j])));
			printf("%s turned %s.\n", as[j].description, (ival(as[j]))? "on" : "off");
			return 0;
		}

		/* Found an abbreviated command, but it wasn't a boolean.
		 * It means it is a user command, to be handled below. */

		if (strcmp(user_command[k].s, "variables") == 0)
		{
			printf(" Variable     Controls                                          Value\n");
			printf(" --------     --------                                          -----\n");
			for (i = 0; as[i].string != NULL; i++)
			{
				printf(" ");
				left_print_string(stdout, as[i].string, 13);
				left_print_string(stdout, as[i].description, 46);
				if (Float == as[i].param_type)
				{
					/* Float point print! */
					printf("%5.2f", *((double *)as[i].ptr));
				}
				else
				if ((Bool == as[i].param_type) || Int == as[i].param_type)
				{
					printf("%5d", ival(as[i]));
				}
				else
				if (String == as[i].param_type)
				{
					printf("%s", *(char **)as[i].ptr);
				}
				if (Bool == as[i].param_type)
				{
					if (ival(as[i])) printf(" (On)"); else printf(" (Off)");
				}
				printf("\n");
			}
			printf("\n");
			printf("Toggle a boolean variable as in \"!batch\"; ");
			printf("set a variable as in \"!width=100\".\n");
			return 0;
		}

		if (strcmp(user_command[k].s, "help") == 0)
		{
			printf("Special commands always begin with \"!\".  Command and variable names\n");
			printf("can be abbreviated.  Here is a list of the commands:\n\n");
			for (i=0; user_command[i].s != NULL; i++) {
				printf(" !");
				left_print_string(stdout, user_command[i].s, 15);
				left_print_string(stdout, user_command[i].str, 52);
				printf("\n");
			}
			printf(" !!<string>      Print all the dictionary words that matches <string>.\n");
			printf("                 A wildcard * may be used to find multiple matches.\n");
			printf("\n");
			printf(" !<var>          Toggle the specified boolean variable.\n");
			printf(" !<var>=<val>    Assign that value to that variable.\n");
			return 0;
		}
	}

	if (s[0] == '!')
	{
		char *out;

		out = dict_display_word_info(dict, s+1, opts);
		if (NULL != out)
		{
			printf("%s\n", out);
		   free(out);
			out = dict_display_word_expr(dict, s+1, opts);
			if (NULL != out)
			{
				printf("%s", out);
				free(out);
			}
			else
			{
				prt_error("Error: '%s': Internal Error: Missing expression.\n", s+1);
			}
		}
		else
		{
			printf("Token \"%s\" matches nothing in the dictionary.\n", s+1);
		}

		return 0;
	}
#ifdef USE_REGEX_TOKENIZER
	if (s[0] == '/')
	{
		int rc = regex_tokenizer_test(dict, s+1);
		if (0 != rc) printf("regex_tokenizer_test: rc %d\n", rc);
		return 0;
	}
#endif

	/* Test here for an equation i.e. does the command line hold an equals sign? */
	for (x=s; (*x != '=') && (*x != '\0') ; x++)
	  ;
	if (*x == '=')
	{
		*x = '\0';
		y = x+1;
		x = s;
		/* now x is the first word and y is the rest */

		/* Figure out which command it is .. it'll be the j'th one */
		j = -1;
		for (i=0; as[i].string != NULL; i++)
		{
			if (strncasecmp(x, as[i].string, strlen(x)) == 0)
			{
				j = i;
				count ++;
			}
		}

		if (j<0)
		{
			printf("There is no user variable called \"%s\".\n", x);
			return -1;
		}

		if (count > 1)
		{
			printf("Ambiguous variable.  Type \"!help\" or \"!variables\"\n");
			return -1;
		}

		if ((as[j].param_type == Int) || (as[j].param_type == Bool))
		{
			int val = -1;
			if (is_numerical_rhs(y)) val = atoi(y);

			if ((0 == strcasecmp(y, "true")) || (0 == strcasecmp(y, "t"))) val = 1;
			if ((0 == strcasecmp(y, "false")) || (0 == strcasecmp(y, "f"))) val = 0;

			if (val < 0)
			{
				printf("Invalid value %s for variable %s Type \"!help\" or \"!variables\"\n", y, as[j].string);
				return -1;
			}

			setival(as[j], val);
			printf("%s set to %d\n", as[j].string, val);
			return 0;
		}
		else
		if (as[j].param_type == Float)
		{
			double val = -1.0;
			val = atof(y);
			if (val < 0.0)
			{
				printf("Invalid value %s for variable %s Type \"!help\" or \"!variables\"\n", y, as[j].string);
				return -1;
			}

			*((double *) as[j].ptr) = val;
			printf("%s set to %5.2f\n", as[j].string, val);
			return 0;
		}
		else
		if (as[j].param_type == String)
		{
			*((char **) as[j].ptr) = y;
			printf("%s set to %s\n", (char *)as[j].string, y);
			return 0;
		}
		else
		{
			prt_error("Error: Internal error: Unknown variable type %d\n",
			          as[j].param_type);
			return -1;
		}
	}

	/* Look for valid commands, but ones that needed an argument */
	j = -1;
	count = 0;
	for (i = 0; as[i].string != NULL; i++)
	{
		if ((Bool != as[i].param_type) &&
		    strncasecmp(s, as[i].string, strlen(s)) == 0)
		{
			j = i;
			count++;
		}
	}

	if (0 < count)
	{
		printf("Variable \"%s\" requires a value.  Try \"!help\".\n", as[j].string);
		return -1;
	}

	printf("I can't interpret \"%s\" as a command.  Try \"!help\".\n", line);
	return -1;
}

static void put_opts_in_local_vars(Command_Options* copts)
{
	Parse_Options opts = copts->popts;
	local.verbosity = parse_options_get_verbosity(opts);
	local.debug = parse_options_get_debug(opts);
	local.test = parse_options_get_test(opts);
	local.timeout = parse_options_get_max_parse_time(opts);;
	local.memory = parse_options_get_max_memory(opts);;
	local.linkage_limit = parse_options_get_linkage_limit(opts);
	local.islands_ok = parse_options_get_islands_ok(opts);
	local.repeatable_rand = parse_options_get_repeatable_rand(opts);
	local.spell_guess = parse_options_get_spell_guess(opts);
	local.short_length = parse_options_get_short_length(opts);
	local.cost_model = parse_options_get_cost_model_type(opts);
	local.max_cost = parse_options_get_disjunct_cost(opts);
	local.use_cluster_disjuncts = parse_options_get_use_cluster_disjuncts(opts);
	local.use_sat_solver = parse_options_get_use_sat_parser(opts);
	local.use_viterbi = parse_options_get_use_viterbi(opts);

	local.screen_width = copts->screen_width;
	local.echo_on = copts->echo_on;
	local.batch_mode = copts->batch_mode;
	local.panic_mode = copts->panic_mode;
	local.allow_null = copts->allow_null;
	local.display_on = copts->display_on;
	local.display_walls = copts->display_walls;
	local.display_postscript = copts->display_postscript;
	local.display_ps_header = copts->display_ps_header;
	local.display_constituents = copts->display_constituents;

	local.display_bad = copts->display_bad;
	local.display_disjuncts = copts->display_disjuncts;
	local.display_links = copts->display_links;
	local.display_senses = copts->display_senses;

	local.display_morphology = parse_options_get_display_morphology(opts);
}

static void put_local_vars_in_opts(Command_Options* copts)
{
	Parse_Options opts = copts->popts;
	parse_options_set_verbosity(opts, local.verbosity);
	parse_options_set_debug(opts, local.debug);
	parse_options_set_test(opts, local.test);
	parse_options_set_max_parse_time(opts, local.timeout);
	parse_options_set_max_memory(opts, local.memory);
	parse_options_set_linkage_limit(opts, local.linkage_limit);
	parse_options_set_islands_ok(opts, local.islands_ok);
	parse_options_set_repeatable_rand(opts, local.repeatable_rand);
	parse_options_set_spell_guess(opts, local.spell_guess);
	parse_options_set_short_length(opts, local.short_length);
	parse_options_set_cost_model_type(opts, local.cost_model);
	parse_options_set_disjunct_cost(opts, local.max_cost);
	parse_options_set_use_cluster_disjuncts(opts, local.use_cluster_disjuncts);
#ifdef USE_SAT_SOLVER
	parse_options_set_use_sat_parser(opts, local.use_sat_solver);
#endif
	parse_options_set_use_viterbi(opts, local.use_viterbi);
	parse_options_set_display_morphology(opts, local.display_morphology);

	copts->screen_width = local.screen_width;
	copts->echo_on = local.echo_on;
	copts->batch_mode = local.batch_mode;
	copts->panic_mode = local.panic_mode;
	copts->allow_null = local.allow_null;
	copts->display_on = local.display_on;
	copts->display_walls = local.display_walls;
	copts->display_postscript = local.display_postscript;
	copts->display_ps_header = local.display_ps_header;
	copts->display_constituents = local.display_constituents;

	copts->display_bad = local.display_bad;
	copts->display_disjuncts = local.display_disjuncts;
	copts->display_links = local.display_links;
	copts->display_senses = local.display_senses;
}

int issue_special_command(const char * line, Command_Options* opts, Dictionary dict)
{
	int rc;
	Parse_Options save = NULL;

	if (strncmp(line, "panic_", 6) == 0)
	{
		line += 6;
		save = opts->popts;
		opts->popts = opts->panic_opts;
	}

	put_opts_in_local_vars(opts);
	char *cline = strdup(line);
	rc = x_issue_special_command(cline, opts, dict);
	put_local_vars_in_opts(opts);
	free(cline);

	if (save) opts->popts = save;
	return rc;
}


Command_Options* command_options_create(void)
{
	Command_Options* co = malloc(sizeof (Command_Options));
	co->popts = parse_options_create();
	co->panic_opts = parse_options_create();

	/* "Unlimited" screen width when writing to a file, auto-updated
	 * later, wen writing to a tty. */
	co->screen_width = 16381;
	co->allow_null = true;
	co->batch_mode = false;
	co->echo_on = false;
	co->panic_mode = false;
	co->display_on = true;
	co->display_walls = false;
	co->display_postscript = false;
	co->display_ps_header = false;
	co->display_constituents = NO_DISPLAY;

	co->display_bad = false;
	co->display_disjuncts = false;
	co->display_links = false;
	co->display_senses = false;
	return co;
}

void command_options_delete(Command_Options* co)
{
	parse_options_delete(co->panic_opts);
	parse_options_delete(co->popts);
	free(co);
}

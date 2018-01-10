/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2013 Linas Vepstas                                          */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/
#ifndef _LINKINCLUDESH_
#define _LINKINCLUDESH_

#ifndef SWIG
#include <stdbool.h> /* Needed for bool typedef */
#include <stdio.h>   /* Needed for FILE* below */
#include <link-grammar/link-features.h>

LINK_BEGIN_DECLS

#ifndef __bool_true_false_are_defined
	#ifdef _Bool
		#define bool                        _Bool
	#else
		#define bool                        int
	#endif
	#define true                            1
	#define false                           0
	#define __bool_true_false_are_defined   1
#endif
#endif /* !SWIG */

/**********************************************************************
 *
 * System initialization
 *
 ***********************************************************************/

typedef struct Dictionary_s * Dictionary;

link_public_api(const char *)
	linkgrammar_get_version(void);

link_public_api(const char *)
	linkgrammar_get_dict_version(Dictionary);

link_public_api(const char *)
	linkgrammar_get_dict_locale(Dictionary);

/**********************************************************************
 *
 * Functions and definitions for the error handler.
 *
 ***********************************************************************/
typedef enum
{
	lg_Fatal = 1,
	lg_Error,
	lg_Warn,
	lg_Info,
	lg_Debug,
	lg_Trace,
	lg_None
} lg_error_severity;

/* Raw error message. */
typedef struct
{
	/* err_ctxt ec; */
	lg_error_severity severity;
	const char *severity_label;
	const char *text;
} lg_errinfo;

/* Error handler callback function. */
typedef void (*lg_error_handler)(lg_errinfo *, void *);

link_public_api(lg_error_handler)
     lg_error_set_handler(lg_error_handler, void *data);
link_public_api(const void *)
     lg_error_set_handler_data(void * data);
link_public_api(char *)
     lg_error_formatmsg(lg_errinfo *lge);
link_public_api(int)
     lg_error_printall(lg_error_handler, void *data);
link_public_api(int)
     lg_error_clearall(void);
link_public_api(bool)
     lg_error_flush(void);

/**********************************************************************
 *
 * Functions to manipulate Dictionaries
 *
 ***********************************************************************/

link_public_api(Dictionary)
     dictionary_create_lang(const char * lang);
link_public_api(Dictionary)
     dictionary_create_default_lang(void);
link_public_api(const char *)
     dictionary_get_lang(Dictionary);

link_public_api(void)
     dictionary_delete(Dictionary);

link_public_api(void)
     dictionary_set_data_dir(const char * path);
link_public_api(char *)
     dictionary_get_data_dir(void);

/**********************************************************************
 *
 * Functions to manipulate Parse Options
 *
 ***********************************************************************/

typedef enum
{
	VDAL=1, /* Sort by Violations, Disjunct cost, Link cost */
	CORPUS, /* Sort by Corpus cost */
} Cost_Model_type;

typedef struct Parse_Options_s * Parse_Options;

link_public_api(Parse_Options)
     parse_options_create(void);
link_public_api(int)
     parse_options_delete(Parse_Options opts);
link_public_api(void)
     parse_options_set_verbosity(Parse_Options opts, int verbosity);
link_public_api(int)
     parse_options_get_verbosity(Parse_Options opts);
link_public_api(void)
     parse_options_set_debug(Parse_Options opts, const char * debug);
link_public_api(char *)
     parse_options_get_debug(Parse_Options opts);
link_public_api(void)
     parse_options_set_test(Parse_Options opts, const char * test);
link_public_api(char *)
     parse_options_get_test(Parse_Options opts);
link_public_api(void)
     parse_options_set_linkage_limit(Parse_Options opts, int linkage_limit);
link_public_api(int)
     parse_options_get_linkage_limit(Parse_Options opts);
link_public_api(void)
     parse_options_set_disjunct_cost(Parse_Options opts, double disjunct_cost);
link_public_api(double)
     parse_options_get_disjunct_cost(Parse_Options opts);
link_public_api(void)
     parse_options_set_min_null_count(Parse_Options opts, int null_count);
link_public_api(int)
     parse_options_get_min_null_count(Parse_Options opts);
link_public_api(void)
     parse_options_set_max_null_count(Parse_Options opts, int null_count);
link_public_api(int)
     parse_options_get_max_null_count(Parse_Options opts);
link_public_api(void)
     parse_options_set_islands_ok(Parse_Options opts, bool islands_ok);
link_public_api(bool)
     parse_options_get_islands_ok(Parse_Options opts);
link_public_api(void)
     parse_options_set_spell_guess(Parse_Options opts, int spell_guess);
link_public_api(int)
     parse_options_get_spell_guess(Parse_Options opts);
link_public_api(void)
     parse_options_set_short_length(Parse_Options opts, int short_length);
link_public_api(int)
     parse_options_get_short_length(Parse_Options opts);
link_public_api(void)
     parse_options_set_max_memory(Parse_Options  opts, int mem);
link_public_api(int)
     parse_options_get_max_memory(Parse_Options opts);
link_public_api(void)
     parse_options_set_max_parse_time(Parse_Options  opts, int secs);
link_public_api(int)
     parse_options_get_max_parse_time(Parse_Options opts);
link_public_api(void)
     parse_options_set_cost_model_type(Parse_Options opts, Cost_Model_type cm);
link_public_api(Cost_Model_type)
     parse_options_get_cost_model_type(Parse_Options opts);
link_public_api(void)
     parse_options_set_perform_pp_prune(Parse_Options opts, bool pp_prune);
link_public_api(bool)
     parse_options_get_perform_pp_prune(Parse_Options opts);
link_public_api(void)
     parse_options_set_use_sat_parser(Parse_Options opts, bool use_sat_solver);
link_public_api(bool)
     parse_options_get_use_sat_parser(Parse_Options opts);
link_public_api(void)
     parse_options_set_use_viterbi(Parse_Options opts, bool use_viterbi);
link_public_api(bool)
     parse_options_get_use_viterbi(Parse_Options opts);
link_public_api(bool)
     parse_options_timer_expired(Parse_Options opts);
link_public_api(bool)
     parse_options_memory_exhausted(Parse_Options opts);
link_public_api(bool)
     parse_options_resources_exhausted(Parse_Options opts);
link_public_api(void)
     parse_options_set_use_cluster_disjuncts(Parse_Options opts, bool val);
link_public_api(bool)
     parse_options_get_use_cluster_disjuncts(Parse_Options opts);
link_public_api(void)
     parse_options_set_all_short_connectors(Parse_Options opts, bool val);
link_public_api(bool)
     parse_options_get_all_short_connectors(Parse_Options opts);
link_public_api(void)
     parse_options_set_repeatable_rand(Parse_Options opts, bool val);
link_public_api(bool)
     parse_options_get_repeatable_rand(Parse_Options opts);
link_public_api(void)
     parse_options_reset_resources(Parse_Options opts);


/**********************************************************************
 *
 * The following Parse_Options functions do not directly affect the
 * operation of the parser, but they can be useful for organizing the
 * search, or displaying the results.  They were included as switches for
 * convenience in implementing the "standard" version of the link parser
 * using the API.
 *
 ***********************************************************************/

typedef enum
{
	NO_DISPLAY = 0,        /** Display is disabled */
	MULTILINE = 1,         /** multi-line, indented display */
	BRACKET_TREE = 2,      /** single-line, bracketed tree */
	SINGLE_LINE = 3,       /** single line, round parenthesis */
   MAX_STYLES = 3         /* this must always be last, largest */
} ConstituentDisplayStyle;


link_public_api(void)
     parse_options_set_display_morphology(Parse_Options opts, int val);
link_public_api(int)
     parse_options_get_display_morphology(Parse_Options opts);

/**********************************************************************
 *
 * Functions to manipulate Sentences
 *
 ***********************************************************************/

typedef struct Sentence_s * Sentence;
typedef size_t LinkageIdx;

link_public_api(Sentence)
     sentence_create(const char *input_string, Dictionary dict);
link_public_api(void)
     sentence_delete(Sentence sent);
link_public_api(int)
     sentence_split(Sentence sent, Parse_Options opts);
link_public_api(int)
     sentence_parse(Sentence sent, Parse_Options opts);
link_public_api(int)
     sentence_length(Sentence sent);
link_public_api(int)
     sentence_null_count(Sentence sent);
link_public_api(int)
     sentence_num_linkages_found(Sentence sent);
link_public_api(int)
     sentence_num_valid_linkages(Sentence sent);
link_public_api(int)
     sentence_num_linkages_post_processed(Sentence sent);
link_public_api(int)
     sentence_num_violations(Sentence sent, LinkageIdx linkage_num);
link_public_api(double)
     sentence_disjunct_cost(Sentence sent, LinkageIdx linkage_num);
link_public_api(int)
     sentence_link_cost(Sentence sent, LinkageIdx linkage_num);

/**********************************************************************
 *
 * Functions that create and manipulate Linkages.
 * When a Linkage is requested, the user is given a
 * copy of all of the necessary information, and is responsible
 * for freeing up the storage when he/she is finished, using
 * the routines provided below.
 *
 ***********************************************************************/

typedef struct Linkage_s * Linkage;
typedef size_t WordIdx;
typedef size_t LinkIdx;

link_public_api(Linkage)
     linkage_create(LinkageIdx linkage_num, Sentence sent, Parse_Options opts);
link_public_api(void)
     linkage_delete(Linkage linkage);
link_public_api(size_t)
     linkage_get_num_words(const Linkage linkage);
link_public_api(size_t)
     linkage_get_num_links(const Linkage linkage);
link_public_api(WordIdx)
     linkage_get_link_lword(const Linkage linkage, LinkIdx index);
link_public_api(WordIdx)
     linkage_get_link_rword(const Linkage linkage, LinkIdx index);
link_public_api(int)
     linkage_get_link_length(const Linkage linkage, LinkIdx index);
link_public_api(const char *)
     linkage_get_link_label(const Linkage linkage, LinkIdx index);
link_public_api(const char *)
     linkage_get_link_llabel(const Linkage linkage, LinkIdx index);
link_public_api(const char *)
     linkage_get_link_rlabel(const Linkage linkage, LinkIdx index);
link_public_api(int)
     linkage_get_link_num_domains(const Linkage linkage, LinkIdx index);
link_public_api(const char **)
     linkage_get_link_domain_names(const Linkage linkage, LinkIdx index);
link_public_api(const char **)
     linkage_get_words(const Linkage linkage);
link_public_api(const char *)
     linkage_get_disjunct_str(const Linkage linkage, WordIdx word_num);
link_public_api(double)
     linkage_get_disjunct_cost(const Linkage linkage, WordIdx word_num);
link_public_api(double)
     linkage_get_disjunct_corpus_score(const Linkage linkage, WordIdx word_num);
link_public_api(const char *)
     linkage_get_word(const Linkage linkage, WordIdx word_num);
link_public_api(char *)
     linkage_print_constituent_tree(Linkage linkage, ConstituentDisplayStyle mode);
link_public_api(void)
     linkage_free_constituent_tree_str(char *str);
link_public_api(char *)
     linkage_print_diagram(const Linkage linkage, bool display_walls, size_t screen_width);
link_public_api(void)
     linkage_free_diagram(char * str);
link_public_api(char *)
     linkage_print_postscript(const Linkage linkage, bool display_walls, bool print_ps_header);
link_public_api(void)
     linkage_free_postscript(char * str);
link_public_api(char *)
     linkage_print_disjuncts(const Linkage linkage);
link_public_api(void)
     linkage_free_disjuncts(char *str);
link_public_api(char *)
     linkage_print_links_and_domains(const Linkage linkage);
link_public_api(void)
     linkage_free_links_and_domains(char *str);
link_public_api(char *)
     linkage_print_pp_msgs(Linkage linkage);
link_public_api(void)
     linkage_free_pp_msgs(char * str);
link_public_api(char *)
     linkage_print_senses(Linkage linkage);
link_public_api(void)
     linkage_free_senses(char *str);
link_public_api(int)
     linkage_unused_word_cost(const Linkage linkage);
link_public_api(double)
     linkage_disjunct_cost(const Linkage linkage);
link_public_api(int)
     linkage_link_cost(const Linkage linkage);
link_public_api(double)
     linkage_corpus_cost(const Linkage linkage);
link_public_api(const char *)
     linkage_get_violation_name(const Linkage linkage);

/* Experimental API - subject to changes. */
link_public_api(size_t)
     linkage_get_word_byte_start(const Linkage linkage, WordIdx w);
link_public_api(size_t)
     linkage_get_word_byte_end(const Linkage linkage, WordIdx w);
link_public_api(size_t)
     linkage_get_word_char_start(const Linkage linkage, WordIdx w);
link_public_api(size_t)
     linkage_get_word_char_end(const Linkage linkage, WordIdx w);


/**********************************************************************
 *
 * Internal functions -- do not use these in new code!
 * They are used in the link-parser code (which needs their signature to
 * compile), but are not intended for general public use.
 *
 ***********************************************************************/

link_public_api(size_t)
     utf8_strwidth(const char *);

link_public_api(char *)
     dict_display_word_expr(Dictionary dict, const char *, Parse_Options opts);
link_public_api(char *)
     dict_display_word_info(Dictionary dict, const char *, Parse_Options opts);
link_public_api(bool)
     lg_expand_disjunct_list(Sentence sent);

/**********************************************************************
 *
 * Internal functions -- do not use these in new code!
 * These are not intended for general public use, but are required to
 * work around certain Microsoft Windows linking oddities
 * (specifically, to be callable from the JNI bindings library.)
 *
 ***********************************************************************/

link_public_api(void)
     parse_options_print_total_time(Parse_Options opts);

#if     __GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ > 4)
#define GNUC_PRINTF( format_idx, arg_idx )    \
  __attribute__((__format__ (__printf__, format_idx, arg_idx)))
#else
#define GNUC_PRINTF( format_idx, arg_idx )
#endif

link_public_api(int)
     prt_error(const char *fmt, ...) GNUC_PRINTF(1,2);

/*******************************************************
 *
 * Macros for marking obsolete functions -- but there aren't any,
 * just right now.
 *
 ********************************************************/

#if  __GNUC__ > 2
#define GNUC_DEPRECATED __attribute__((deprecated))
#else
#define GNUC_DEPRECATED
#endif

#if defined(_MSC_VER)
 #define MS_DEPRECATED __declspec(deprecated)
#else
 #define MS_DEPRECATED
#endif

#ifndef SWIG
LINK_END_DECLS
#endif /* !SWIG */

#endif

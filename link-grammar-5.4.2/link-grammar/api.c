/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2008, 2009, 2013, 2014 Linas Vepstas                        */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <limits.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "api-structures.h"
#include "connectors.h"  // for MAX_SENTENCE
#include "corpus/corpus.h"
#include "dict-common/dict-common.h"
#include "dict-common/dict-utils.h" // for free_X_nodes
#include "disjunct-utils.h"  // for free_disjuncts
#include "error.h"
#include "externs.h"
#include "linkage/linkage.h"
#include "parse/histogram.h"  // for PARSE_NUM_OVERFLOW
#include "parse/parse.h"
#include "post-process/post-process.h" // for post_process_new()
#include "print/print.h"
#include "prepare/exprune.h"
#include "resources.h"
#include "sat-solver/sat-encoder.h"
#include "string-set.h"
#include "tokenize/spellcheck.h"
#include "tokenize/tokenize.h"
#include "tokenize/tok-structures.h" // Needed for Gword_struct
#include "tokenize/word-structures.h" // Needed for Word_struct/free_X_node
#include "utilities.h"

/* Its OK if this is racey across threads.  Any mild shuffling is enough. */
static unsigned int global_rand_state = 0;

int verbosity;
/* debug and test should not be NULL since they can be used before they
 * are assigned a value by parse_options_get_...() */
char * debug = (char *)"";
char * test = (char *)"";

/***************************************************************
*
* Routines for setting Parse_Options
*
****************************************************************/

/**
 * For sorting the linkages in postprocessing
 */

static int VDAL_compare_parse(Linkage l1, Linkage l2)
{
	Linkage_info * p1 = &l1->lifo;
	Linkage_info * p2 = &l2->lifo;

	/* Move the discarded entries to the end of the list */
	if (p1->discarded || p2->discarded) return (p1->discarded - p2->discarded);

	if (p1->N_violations != p2->N_violations) {
		return (p1->N_violations - p2->N_violations);
	}
	else if (p1->unused_word_cost != p2->unused_word_cost) {
		return (p1->unused_word_cost - p2->unused_word_cost);
	}
	else if (p1->disjunct_cost > p2->disjunct_cost) return 1;
	else if (p1->disjunct_cost < p2->disjunct_cost) return -1;
	else {
		return (p1->link_cost - p2->link_cost);
	}
}

#ifdef USE_CORPUS
static int CORP_compare_parse(Linkage l1, Linkage l2)
{
	Linkage_info * p1 = &l1->lifo;
	Linkage_info * p2 = &l2->lifo;

	double diff = p1->corpus_cost - p2->corpus_cost;

	/* Move the discarded entries to the end of the list */
	if (p1->discarded || p2->discarded) return (p1->discarded - p2->discarded);

	if (fabs(diff) < 1.0e-5)
		return VDAL_compare_parse(p1, p2);
	if (diff < 0.0) return -1;
	return 1;
}
#endif

/**
 * Create and initialize a Parse_Options object
 */
Parse_Options parse_options_create(void)
{
	Parse_Options po;

	init_memusage();
	po = (Parse_Options) xalloc(sizeof(struct Parse_Options_s));

	/* Here's where the values are initialized */

	/* The parse_options_set_(verbosity|debug|test) functions set also the
	 * corresponding global variables. So these globals are initialized
	 * here too. */
	verbosity = po->verbosity = 1;
	debug = po->debug = (char *)"";
	test = po->test = (char *)"";

	/* A cost of 2.7 allows the usual cost-2 connectors, plus the
	 * assorted fractional costs, without going to cost 3.0, which
	 * is used only during panic-parsing.
	 * XXX In the long run, this should be fetched from the dictionary
	 * (and should probably not be a parse option).
	 */
	po->disjunct_cost = 2.7;
	po->min_null_count = 0;
	po->max_null_count = 0;
	po->islands_ok = false;
	po->use_sat_solver = false;
	po->use_viterbi = false;
	po->linkage_limit = 100;
#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
	po->use_spell_guess = 7;
#else
	po->use_spell_guess = 0;
#endif /* defined HAVE_HUNSPELL || defined HAVE_ASPELL */

#ifdef XXX_USE_CORPUS
	/* Use the corpus cost model, if available.
	 * It really does a better job at parse ranking.
	 * Err .. sometimes ...
	 */
	po->cost_model.compare_fn = &CORP_compare_parse;
	po->cost_model.type = CORPUS;
#else /* USE_CORPUS */
	po->cost_model.compare_fn = &VDAL_compare_parse;
	po->cost_model.type = VDAL;
#endif /* USE_CORPUS */
	po->short_length = 16;
	po->all_short = false;
	po->perform_pp_prune = true;
	po->twopass_length = 30;
	po->repeatable_rand = true;
	po->resources = resources_create();
	po->use_cluster_disjuncts = false;
	po->display_morphology = false;

	return po;
}

int parse_options_delete(Parse_Options  opts)
{
	resources_delete(opts->resources);
	xfree(opts, sizeof(struct Parse_Options_s));
	return 0;
}

void parse_options_set_cost_model_type(Parse_Options opts, Cost_Model_type cm)
{
	switch(cm) {
	case VDAL:
		opts->cost_model.type = VDAL;
		opts->cost_model.compare_fn = &VDAL_compare_parse;
		break;
	case CORPUS:
#ifdef USE_CORPUS
		opts->cost_model.type = CORPUS;
		opts->cost_model.compare_fn = &CORP_compare_parse;
#else
		prt_error("Error: Source code compiled with cost model 'CORPUS' disabled.\n");
#endif
		break;
	default:
		prt_error("Error: Illegal cost model: %d\n", cm);
	}
}

Cost_Model_type parse_options_get_cost_model_type(Parse_Options opts)
{
	return opts->cost_model.type;
}

void parse_options_set_perform_pp_prune(Parse_Options opts, bool dummy)
{
	opts->perform_pp_prune = dummy;
}

bool parse_options_get_perform_pp_prune(Parse_Options opts) {
	return opts->perform_pp_prune;
}

void parse_options_set_verbosity(Parse_Options opts, int dummy)
{
	opts->verbosity = dummy;
	verbosity = opts->verbosity;
	/* this is one of the only global variables. */
}

int parse_options_get_verbosity(Parse_Options opts) {
	return opts->verbosity;
}

void parse_options_set_debug(Parse_Options opts, const char * dummy)
{
	/* The comma-separated list of functions is limited to this size.
	 * Can be easily dynamically allocated. In any case it is not reentrant
	 * because the "debug" variable is static. */
	static char buff[256];
	size_t len = strlen(dummy);

	if (0 == strcmp(dummy, opts->debug)) return;

	if (0 == len)
	{
		buff[0] = '\0';
	}
	else
	{
		buff[0] = ',';
		strncpy(buff+1, dummy, sizeof(buff)-2);
		if (len < sizeof(buff)-2)
		{
			buff[len+1] = ',';
			buff[len+2] = '\0';
		}
		else
		{
			buff[sizeof(buff)-1] = '\0';
		}
	}
	opts->debug = buff;
	debug = opts->debug;
	/* this is one of the only global variables. */
}

char * parse_options_get_debug(Parse_Options opts) {
	return opts->debug;
}

void parse_options_set_test(Parse_Options opts, const char * dummy)
{
	/* The comma-separated test features is limited to this size.
	 * Can be easily dynamically allocated. In any case it is not reentrant
	 * because the "test" variable is static. */
	static char buff[256];
	size_t len = strlen(dummy);

	if (0 == strcmp(dummy, opts->test)) return;

	if (0 == len)
	{
		buff[0] = '\0';
	}
	else
	{
		buff[0] = ',';
		strncpy(buff+1, dummy, sizeof(buff)-2);
		if (len < sizeof(buff)-2)
		{
			buff[len+1] = ',';
			buff[len+2] = '\0';
		}
		else
		{
			buff[sizeof(buff)-1] = '\0';
		}
	}
	opts->test = buff;
	test = opts->test;
	/* this is one of the only global variables. */
}

char * parse_options_get_test(Parse_Options opts) {
	return opts->test;
}

void parse_options_set_use_sat_parser(Parse_Options opts, bool dummy) {
#ifdef USE_SAT_SOLVER
	opts->use_sat_solver = dummy;
#else
	if (dummy && (verbosity > D_USER_BASIC))
	{
		prt_error("Error: Cannot enable the Boolean SAT parser; "
		          "this library was built without SAT solver support.\n");
	}
#endif
}

bool parse_options_get_use_sat_parser(Parse_Options opts) {
	return opts->use_sat_solver;
}

void parse_options_set_use_viterbi(Parse_Options opts, bool dummy) {
	opts->use_viterbi = dummy;
}

bool parse_options_get_use_viterbi(Parse_Options opts) {
	return opts->use_viterbi;
}

void parse_options_set_linkage_limit(Parse_Options opts, int dummy)
{
	opts->linkage_limit = dummy;
}
int parse_options_get_linkage_limit(Parse_Options opts)
{
	return opts->linkage_limit;
}

void parse_options_set_disjunct_cost(Parse_Options opts, double dummy)
{
	opts->disjunct_cost = dummy;
}
double parse_options_get_disjunct_cost(Parse_Options opts)
{
	return opts->disjunct_cost;
}

void parse_options_set_min_null_count(Parse_Options opts, int val) {
	opts->min_null_count = val;
}
int parse_options_get_min_null_count(Parse_Options opts) {
	return opts->min_null_count;
}

void parse_options_set_max_null_count(Parse_Options opts, int val) {
	opts->max_null_count = val;
}
int parse_options_get_max_null_count(Parse_Options opts) {
	return opts->max_null_count;
}

void parse_options_set_islands_ok(Parse_Options opts, bool dummy) {
	opts->islands_ok = dummy;
}

bool parse_options_get_islands_ok(Parse_Options opts) {
	return opts->islands_ok;
}

void parse_options_set_spell_guess(Parse_Options opts, int dummy) {
#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
	opts->use_spell_guess = dummy;
#else
	if (dummy && (verbosity > D_USER_BASIC))
	{
		prt_error("Error: Cannot enable spell guess; "
		        "this library was built without spell guess support.\n");
	}

#endif /* defined HAVE_HUNSPELL || defined HAVE_ASPELL */
}

int parse_options_get_spell_guess(Parse_Options opts) {
	return opts->use_spell_guess;
}

void parse_options_set_short_length(Parse_Options opts, int short_length) {
	opts->short_length = short_length;
}

int parse_options_get_short_length(Parse_Options opts) {
	return opts->short_length;
}

void parse_options_set_all_short_connectors(Parse_Options opts, bool val) {
	opts->all_short = val;
}

bool parse_options_get_all_short_connectors(Parse_Options opts) {
	return opts->all_short;
}

/** True means "make it repeatable.". False means "make it random". */
void parse_options_set_repeatable_rand(Parse_Options opts, bool val)
{
	opts->repeatable_rand = val;

	/* Zero is used to indicate repeatability. */
	if (val) global_rand_state = 0;
	else if (0 == global_rand_state) global_rand_state = 42;
}

bool parse_options_get_repeatable_rand(Parse_Options opts) {
	return opts->repeatable_rand;
}

void parse_options_set_max_parse_time(Parse_Options opts, int dummy) {
	opts->resources->max_parse_time = dummy;
}

int parse_options_get_max_parse_time(Parse_Options opts) {
	return opts->resources->max_parse_time;
}

void parse_options_set_max_memory(Parse_Options opts, int dummy) {
	opts->resources->max_memory = dummy;
}

int parse_options_get_max_memory(Parse_Options opts) {
	return opts->resources->max_memory;
}

void parse_options_set_use_cluster_disjuncts(Parse_Options opts, bool dummy) {
	opts->use_cluster_disjuncts = dummy;
}

bool parse_options_get_use_cluster_disjuncts(Parse_Options opts) {
	return opts->use_cluster_disjuncts;
}

int parse_options_get_display_morphology(Parse_Options opts) {
	return opts->display_morphology;
}

void parse_options_set_display_morphology(Parse_Options opts, int dummy) {
	opts->display_morphology = dummy;
}

bool parse_options_timer_expired(Parse_Options opts) {
	return resources_timer_expired(opts->resources);
}

bool parse_options_memory_exhausted(Parse_Options opts) {
	return resources_memory_exhausted(opts->resources);
}

bool parse_options_resources_exhausted(Parse_Options opts) {
	return (resources_exhausted(opts->resources));
}

void parse_options_reset_resources(Parse_Options opts) {
	resources_reset(opts->resources);
}

/***************************************************************
*
* Routines for creating destroying and processing Sentences
*
****************************************************************/

Sentence sentence_create(const char *input_string, Dictionary dict)
{
	Sentence sent;

	sent = (Sentence) xalloc(sizeof(struct Sentence_s));
	memset(sent, 0, sizeof(struct Sentence_s));

	sent->dict = dict;
	sent->string_set = string_set_create();
	sent->rand_state = global_rand_state;

	sent->postprocessor = post_process_new(dict->base_knowledge);

	/* Make a copy of the input */
	sent->orig_sentence = string_set_add (input_string, sent->string_set);

	return sent;
}

int sentence_split(Sentence sent, Parse_Options opts)
{
	Dictionary dict = sent->dict;
	bool fw_failed = false;

	/* 0 == global_rand_state denotes "repeatable rand".
	 * If non-zero, set it here so that anysplit can use it.
	 */
	if (false == opts->repeatable_rand && 0 == sent->rand_state)
	{
		if (0 == global_rand_state) global_rand_state = 42;
		sent->rand_state = global_rand_state;
	}

	/* Tokenize */
	if (!separate_sentence(sent, opts))
	{
		return -1;
	}

	/* Flatten the word graph created by separate_sentence() to a 2D-word-array
	 * which is compatible to the current parsers.
	 * This may fail if UNKNOWN_WORD is needed but
	 * is not defined in the dictionary, or an internal error happens. */
	fw_failed = !flatten_wordgraph(sent, opts);

	/* If unknown_word is not defined, then no special processing
	 * will be done for e.g. capitalized words. */
	if (!(dict->unknown_word_defined && dict->use_unknown_word))
	{
		if (!sentence_in_dictionary(sent)) {
			return -2;
		}
	}

	if (fw_failed)
	{
		/* Make sure an error message is always printed.
		 * So it may be redundant. */
		prt_error("Error: sentence_split(): Internal error detected\n");
		return -3;
	}

	return 0;
}

static void free_sentence_words(Sentence sent)
{
	size_t i;

	for (i = 0; i < sent->length; i++)
	{
		free_X_nodes(sent->word[i].x);
		free_disjuncts(sent->word[i].d);
		free(sent->word[i].alternatives);
	}
	free((void *) sent->word);
	sent->word = NULL;
}

// XXX FIXME ... these should find a home in the tokenize directory.
static void wordgraph_delete(Sentence sent)
{
	Gword *w = sent->wordgraph;

	while(NULL != w)
	{
		Gword *w_tofree = w;

		free(w->prev);
		free(w->next);
		free(w->hier_position);
		free(w->null_subwords);
		w = w->chain_next;
		free(w_tofree);
	}
	sent->wordgraph = sent->last_word = NULL;
}

static void word_queue_delete(Sentence sent)
{
	struct word_queue *wq = sent->word_queue;
	while (NULL != wq)
	{
		struct word_queue *wq_tofree = wq;
		wq = wq->next;
		free(wq_tofree);
	};
	sent->word_queue = NULL;
}

/**
 * Delete the gword_set associated with the Wordgraph.
 * @w First Wordgraph word.
 */
static void gword_set_delete(Gword *w)
{
	for (w = w->chain_next; NULL != w; w = w->chain_next)
	{
		gword_set *n;
		for (gword_set *f = w->gword_set_head.chain_next; NULL != f; f = n)
		{
			n = f->chain_next;
			free(f);
		}
	}
}

void sentence_delete(Sentence sent)
{
	if (!sent) return;
	sat_sentence_delete(sent);
	free_sentence_words(sent);
	gword_set_delete(sent->wordgraph);
	wordgraph_delete(sent);
	word_queue_delete(sent);
	string_set_delete(sent->string_set);
	free_linkages(sent);
	post_process_free(sent->postprocessor);
	post_process_free(sent->constituent_pp);

	global_rand_state = sent->rand_state;
	xfree((char *) sent, sizeof(struct Sentence_s));
}

int sentence_length(Sentence sent)
{
	if (!sent) return 0;
	return sent->length;
}

int sentence_null_count(Sentence sent)
{
	if (!sent) return 0;
	return sent->null_count;
}

int sentence_num_linkages_found(Sentence sent)
{
	if (!sent) return 0;
	return sent->num_linkages_found;
}

int sentence_num_valid_linkages(Sentence sent)
{
	if (!sent) return 0;
	return sent->num_valid_linkages;
}

int sentence_num_linkages_post_processed(Sentence sent)
{
	if (!sent) return 0;
	return sent->num_linkages_post_processed;
}

int sentence_num_violations(Sentence sent, LinkageIdx i)
{
	if (!sent) return 0;

	if (!sent->lnkages) return 0;
	if (sent->num_linkages_alloced <= i) return 0; /* bounds check */
	return sent->lnkages[i].lifo.N_violations;
}

double sentence_disjunct_cost(Sentence sent, LinkageIdx i)
{
	if (!sent) return 0.0;

	/* The sat solver (currently) fails to fill in link_info */
	if (!sent->lnkages) return 0.0;
	if (sent->num_linkages_alloced <= i) return 0.0; /* bounds check */
	return sent->lnkages[i].lifo.disjunct_cost;
}

int sentence_link_cost(Sentence sent, LinkageIdx i)
{
	if (!sent) return 0;

	/* The sat solver (currently) fails to fill in link_info */
	if (!sent->lnkages) return 0;
	if (sent->num_linkages_alloced <= i) return 0; /* bounds check */
	return sent->lnkages[i].lifo.link_cost;
}

static void free_sentence_disjuncts(Sentence sent)
{
	size_t i;

	for (i = 0; i < sent->length; ++i)
	{
		free_disjuncts(sent->word[i].d);
		sent->word[i].d = NULL;
	}
}

int sentence_parse(Sentence sent, Parse_Options opts)
{
	int rc;

	sent->num_valid_linkages = 0;

	/* If the sentence has not yet been split, do so now.
	 * This is for backwards compatibility, for existing programs
	 * that do not explicitly call the splitter.
	 */
	if (0 == sent->length)
	{
		rc = sentence_split(sent, opts);
		if (rc) return -1;
	}

	/* Check for bad sentence length */
	if (MAX_SENTENCE <= sent->length)
	{
		prt_error("Error: sentence too long, contains more than %d words\n",
			MAX_SENTENCE);
		return -2;
	}

	/* During a panic parse, we enter here a second time, with leftover
	 * garbage. Free it. We really should make the code that is panicking
	 * do this free, but right now, they have no PAI for it, so we do it
	 * as a favor. XXX FIXME someday. */
	free_sentence_disjuncts(sent);
	resources_reset(opts->resources);

	/* Expressions were set up during the tokenize stage.
	 * Prune them, and then parse.
	 */
	expression_prune(sent);
	print_time(opts, "Finished expression pruning");
	if (opts->use_sat_solver)
	{
		sat_parse(sent, opts);
	}
	else
	{
		classic_parse(sent, opts);
	}
	print_time(opts, "Finished parse");

	if ((verbosity > 0) &&
	   (PARSE_NUM_OVERFLOW < sent->num_linkages_found))
	{
		prt_error("Warning: Combinatorial explosion! nulls=%zu cnt=%d\n"
			"Consider retrying the parse with the max allowed disjunct cost set lower.\n"
			"At the command line, use !cost-max\n",
			sent->null_count, sent->num_linkages_found);
	}
	return sent->num_valid_linkages;
}

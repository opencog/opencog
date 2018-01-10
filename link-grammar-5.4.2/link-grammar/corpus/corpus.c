/*
 * corpus.c
 *
 * Data for corpus statistics, used to provide a parse ranking
 * to drive the SAT solver, as well as parse ranking with the
 * ordinary solver.
 *
 * Copyright (c) 2008, 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sqlite3.h>
#include "corpus.h"

#include "api-structures.h"
#include "disjuncts.h"
#include "utilities.h"

struct corpus_s
{
	char * dbname;
	sqlite3 *dbconn;
	sqlite3_stmt *rank_query;
	sqlite3_stmt *sense_query;
	const char *errmsg;
	int rc;
};

struct sense_s
{
	int word;
	const char * inflected_word;
	const char * disjunct;
	char * sense;
	double score;
	Sense *next;
};

/* ========================================================= */

static void * db_file_open(const char * dbname, void * user_data)
{
	Corpus *c = (Corpus *) user_data;
	int rc;
	sqlite3 *dbconn;
	c->rc = sqlite3_open_v2(dbname, &dbconn, SQLITE_OPEN_READONLY, NULL);
	if (c->rc)
	{
		c->errmsg = sqlite3_errmsg(dbconn);
		sqlite3_close(dbconn);
		return NULL;
	}

	c->dbname = strdup(dbname);
	return dbconn;
}


/**
 * Initialize the corpus statistics subsystem.
 */
Corpus * lg_corpus_new(void)
{
	int rc;

	Corpus *c = (Corpus *) malloc(sizeof(Corpus));
	c->rank_query = NULL;
	c->sense_query = NULL;
	c->errmsg = NULL;
	c->dbname = NULL;

	/* dbname = "/link-grammar/data/en/sql/disjuncts.db"; */
#ifdef _WIN32
#define DBNAME "sql\\disjuncts.db"
#else
#define DBNAME "sql/disjuncts.db"
#endif
	c->dbconn = object_open(DBNAME, db_file_open, c);
	if (NULL == c->dbconn)
	{
		if (SQLITE_CANTOPEN == c->rc)
		{
			prt_error("Warning: File not found: %s\n"
			          "\tWas looking for: " DBNAME "\n",
				c->errmsg);
		}
		else
		{
			prt_error("Warning: Can't open database: %s\n"
			          "\tWas looking for: " DBNAME "\n",
				c->errmsg);
		}
		return c;
	}

	/* Now prepare the statements we plan to use */
	rc = sqlite3_prepare_v2(c->dbconn,
		"SELECT log_cond_probability FROM Disjuncts "
		"WHERE inflected_word = ? AND disjunct = ?;",
		-1, &c->rank_query, NULL);
	if (rc != SQLITE_OK)
	{
		prt_error("Error: Can't prepare the ranking statment: %s\n",
			sqlite3_errmsg(c->dbconn));
	}

	/* Results are returned in sorted order .. would it be faster
	 * to sort locally? Don't know ... */
	rc = sqlite3_prepare_v2(c->dbconn,
		"SELECT word_sense, log_cond_probability FROM DisjunctSenses "
		"WHERE inflected_word = ? AND disjunct = ? "
		"ORDER BY log_cond_probability ASC;",
		-1, &c->sense_query, NULL);
	if (rc != SQLITE_OK)
	{
		prt_error("Error: Can't prepare the sense statment: %s\n",
			sqlite3_errmsg(c->dbconn));
	}

	prt_error("Info: Corpus statistics database found at %s\n", c->dbname);
	return c;
}

/**
 * lg_corpus_delete -- shut down the corpus statistics subsystem.
 */
void lg_corpus_delete(Corpus *c)
{
	if (NULL == c) return;

	if (c->rank_query)
	{
		sqlite3_finalize(c->rank_query);
		c->rank_query = NULL;
	}

	if (c->sense_query)
	{
		sqlite3_finalize(c->sense_query);
		c->sense_query = NULL;
	}

	if (c->dbconn)
	{
		sqlite3_close(c->dbconn);
		c->dbconn = NULL;
	}

	if (c->dbname)
	{
		free(c->dbname);
		c->dbname = NULL;
	}
	free(c);
}

/* ========================================================= */

/* LOW_SCORE is what is assumed if a disjunct-word pair is not found
 * in the dictionary. It is meant to be -log_2(prob(d|w)) where
 * prob(d|w) is the conditional probability of seeing the disjunct d
 * given the word w. A value of 17 is about equal to 1 in 100,000.
 */
#define LOW_SCORE 17.0

/**
 * get_disjunct_score -- get log probability of observing disjunt.
 *
 * Given an "inflected" word and a disjunct, this routine returns the
 * -log_2 conditional probability prob(d|w) of seeing the disjunct 'd'
 * given that the word 'w' was observed.  Here, "inflected word" means
 * the link-grammar dictionary entry, complete with its trailing period
 * and tag -- e.g. run.v or running.g -- everything after the dot is the
 * "inflection".
 */
static double get_disjunct_score(Corpus *corp,
                                 const char * inflected_word,
                                 const char * disjunct)
{
	double val;
	int rc;

	/* Look up the disjunct in the database */
	rc = sqlite3_bind_text(corp->rank_query, 1,
		inflected_word, -1, SQLITE_STATIC);
	if (rc != SQLITE_OK)
	{
		const char *errmsg = sqlite3_errmsg(corp->dbconn);
		prt_error("Error: SQLite can't bind word: rc=%d %s\n", rc, errmsg);
		return LOW_SCORE;
	}

	rc = sqlite3_bind_text(corp->rank_query, 2,
		disjunct, -1, SQLITE_STATIC);
	if (rc != SQLITE_OK)
	{
		const char *errmsg = sqlite3_errmsg(corp->dbconn);
		prt_error("Error: SQLite can't bind disjunct: rc=%d %s\n", rc, errmsg);
		return LOW_SCORE;
	}

	rc = sqlite3_step(corp->rank_query);
	if (rc != SQLITE_ROW)
	{
		val = LOW_SCORE;
#ifdef DEBUG
		printf ("Word=%s dj=%s not found in dict, assume score=%f\n",
			inflected_word, disjunct, val);
#endif
		if (rc < SQLITE_ROW)
		{
			const char *errmsg = sqlite3_errmsg(corp->dbconn);
			prt_error("Error: SQLite can't ifind word: rc=%d %s\n", rc, errmsg);
		}
	}
	else
	{
		val = sqlite3_column_double(corp->rank_query, 0);
		if (LOW_SCORE < val) val = LOW_SCORE;
#ifdef DEBUG
		printf ("Word=%s dj=%s score=%f\n", inflected_word, disjunct, val);
#endif
	}

	/* Failure to do both a reset *and* a clear will cause subsequent
	 * binds to fail. */
	sqlite3_reset(corp->rank_query);
	sqlite3_clear_bindings(corp->rank_query);
	return val;
}

/* ========================================================= */

/**
 * lg_corpus_score -- compute parse-ranking score for sentence.
 *
 * Given a parsed sentence, this routine will compute a parse ranking
 * score, based on the probabilities of observing the indicated set of
 * disjuncts in the statistics database.
 *
 * The score is stored in the Linkage_info->corpus_cost struct member.
 *
 * The score is currently computed as the average -log_2 conditional
 * probability p(d|w) of observing disjunct 'd', given word 'w'.
 * Lower scores are better -- they indicate more likely parses.
 */
void lg_corpus_score(Linage lkg)
{
	const char *infword, *djstr;
	double tot_score = 0.0f;
	Corpus *corp = sent->dict->corpus;
	int nwords = lkg->num_words;
	int w;

	/* No-op if the database is not open */
	if (NULL == corp->dbconn) return;

	lg_compute_disjunct_strings(lkg);

	/* Decrement nwords, so as to ignore the RIGHT-WALL */
	nwords --;

	/* Loop over each word in the sentence (skipping LEFT-WALL, which is
	 * word 0. */
	for (w=1; w<nwords; w++)
	{
		Disjunct *disj = lkg->chosen_disjuncts[w];

		/* disj is NULL if word did not participate in parse */
		if (NULL == disj)
		{
			tot_score += LOW_SCORE;
			continue;
		}
		infword = disj->string;
		djstr = lkg->disjunct_list_str[w];
		tot_score += get_disjunct_score(corp, infword, djstr);
	}

	/* Decrement nwords, so as to ignore the LEFT-WALL */
	--nwords;
	tot_score /= nwords;
	lkg->lifo.corpus_cost = tot_score;
}

double lg_corpus_disjunct_score(Linkage linkage, int w)
{
	double score;
	const char *infword, *djstr;
	Sentence sent = linkage->sent;
	Corpus *corp = sent->dict->corpus;
	Disjunct *disj;

	/* No-op if the database is not open */
	if (NULL == corp->dbconn) return LOW_SCORE;

	/* disj is NULL if word did not participate in parse */
	disj = linkage->chosen_disjuncts[w];
	if (NULL == disj) return LOW_SCORE;

	lg_compute_disjunct_strings(linkage);

	infword = disj->string;
	djstr = linkage->disjunct_list_str[w];
	score = get_disjunct_score(corp, infword, djstr);

	return score;
}

/* ========================================================= */

/**
 * lg_corpus_senses -- Given word and disjunct, look up senses.
 *
 * Given a particular disjunct for a word, look up its most
 * likely sense assignments from the database.
 */

static Sense * lg_corpus_senses(Corpus *corp,
                                const char * inflected_word,
                                const char * disjunct,
                                int wrd)
{
	double log_prob;
	const unsigned char *sense;
	Sense *sns, *head = NULL;
	int rc;

	/* Look up the disjunct in the database */
	rc = sqlite3_bind_text(corp->sense_query, 1,
		inflected_word, -1, SQLITE_STATIC);
	if (rc != SQLITE_OK)
	{
		prt_error("Error: SQLite can't bind word in sense query: rc=%d \n", rc);
		return NULL;
	}

	rc = sqlite3_bind_text(corp->sense_query, 2,
		disjunct, -1, SQLITE_STATIC);
	if (rc != SQLITE_OK)
	{
		prt_error("Error: SQLite can't bind disjunct in sense query: rc=%d \n", rc);
		return NULL;
	}

	rc = sqlite3_step(corp->sense_query);
	while (SQLITE_ROW == rc)
	{
		sense = sqlite3_column_text(corp->sense_query, 0);
		log_prob = sqlite3_column_double(corp->sense_query, 1);
		// printf ("Word=%s dj=%s sense=%s score=%f\n",
		//    inflected_word, disjunct, sense, log_prob);

		sns = (Sense *) malloc(sizeof(Sense));
		sns->next = head;
		head = sns;

		sns->inflected_word = inflected_word;
		sns->disjunct = disjunct;
		sns->sense = strdup(sense);
		sns->score = log_prob;
		sns->word = wrd;

		/* Get the next row, if any */
		rc = sqlite3_step(corp->sense_query);
	}

	/* Failure to do both a reset *and* a clear will cause subsequent
	 * binds tp fail. */
	sqlite3_reset(corp->sense_query);
	sqlite3_clear_bindings(corp->sense_query);

	return head;
}

/* ========================================================= */

/**
 * lg_corpus_linkage_senses -- Given a linkage, look up senses.
 *
 * Given a particular linkage, look up the most likely sense
 * assignments from the database.
 *
 * This function is not used to guide the parsing process; it is
 * only an informational look-up.
 */

void lg_corpus_linkage_senses(Linkage lkg)
{
	const char * infword;
	Sentence sent = lkg->sent;
	Dictionary dict = sent->dict;
	Corpus *corp = dict->corpus;
	int nwords = lkg->num_words;
	int w;

	if (lkg->sense_list) return;

	/* Set up the disjunct strings first */
	lg_compute_disjunct_strings(lkg);

	lkg->sense_list = (Sense **) malloc(nwords * sizeof (Sense *));
	memset(lkg->sense_list, 0, nwords * sizeof (Sense *));

	/* Decrement nwords, so as to ignore the RIGHT-WALL */
	nwords --;

	/* Loop over each word in the sentence (skipping LEFT-WALL, which is
	 * word 0. */
	for (w=1; w<nwords; w++)
	{
		Disjunct *disj = lkg->chosen_disjuncts[w];

		/* disj is NULL if word did not participate in parse */
		if (NULL == disj)
		{
			continue;
		}
		infword = disj->string;

		lkg->sense_list[w] = lg_corpus_senses(corp, infword,
		                       lkg->disjunct_list_str[w], w);
	}
}

/* ========================================================= */
/* Return bits and pieces of the sense assignments */

Sense * lg_get_word_sense(Linkage lkg, WordIdx word)
{
	if (!lkg->sense_list) return NULL;
	if (lkg->num_words <= word) return NULL;
	return lkg->sense_list[word];
}

Sense * lg_sense_next(Sense *sns)
{
	return sns->next;
}

int lg_sense_get_index(Sense *sns)
{
	return sns->word;
}

const char * lg_sense_get_subscripted_word(Sense *sns)
{
	return sns->inflected_word;
}

const char * lg_sense_get_disjunct(Sense *sns)
{
	return sns->disjunct;
}

const char * lg_sense_get_sense(Sense *sns)
{
	return sns->sense;
}

double lg_sense_get_score(Sense *sns)
{
	return sns->score;
}

void lg_sense_delete(Linkage lkg)
{
	size_t nwords = lkg->num_words;
	size_t w;

	if (NULL == lkg->sense_list) return;

	for (w=0; w<nwords; w++)
	{
		Sense *sns = lkg->sense_list[w];
		while (sns)
		{
			Sense * nxt = sns->next;
			free(sns->sense);
			free(sns);
			sns = nxt;
		}
	}
	free (lkg->sense_list);
	lkg->sense_list = NULL;
}

/* ======================= END OF FILE ===================== */

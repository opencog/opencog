/*
 * EdgeUtils.cc
 *
 * Utilities for counting, manipulating edges between words in a sentence.
 *
 * Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
 */

#include "EdgeUtils.h"
#include "ForeachWord.h"

#define DEBUG

using namespace opencog;

/**
 * For each word-instance loop over all syntactic relationships.
 * (i.e. _subj, _obj, _nn, _amod, and so on). Generate a list of all
 * words that participate in relationships.
 */
bool EdgeUtils::look_at_word(const Handle& h)
{
	foreach_relex_relation(h, &EdgeUtils::look_at_relation, this);
	return false;
}

/**
 * This routine is called for every relation between word-instances in
 * a parse. It simply creates a list of all of the words in a sentence
 * that participate in RelEx relations.
 */
bool EdgeUtils::look_at_relation(const std::string &relname,
                             const Handle& first, const Handle& second)
{
	words.insert(first);
	words.insert(second);
	return false;
}


/*
 * Mihalcea.cc
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "ForeachChaseLink.h"
#include "Mihalcea.h"

using namespace opencog;

Mihalcea::Mihalcea(void)
{
}

Mihalcea::~Mihalcea()
{
}

/**
 * Anotate every word in every parse of the sentence with every possible
 * word sense for that word, given its part-of-speech.
 */
void Mihalcea::annotate_sentence(Handle h)
{
	ForeachChaseLink<Mihalcea> chase;
	chase.backtrack_binary_link(h, PARSE_LINK, &Mihalcea::annotate_parse, this);
}

/**
 * Anotate every word in the given parse with every possible
 * word sense for that word, given its part-of-speech.
 */
bool Mihalcea::annotate_parse(Handle h)
{
	printf("found parse %x\n", (unsigned long) h);
	return false;
}

void Mihalcea::process_sentence(Handle h)
{
	annotate_sentence(h);
}


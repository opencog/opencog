/*
 * SenseSimilaritySQL.cc
 *
 * Fetches wordnet-based sense-similarity measures from database.
 * These had been previous pre-computed.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include "platform.h"
#include <stdio.h>
#include <math.h>

#include "ForeachWord.h"
#include "SenseSimilaritySQL.h"
#include "Node.h"
#include "SimpleTruthValue.h"

using namespace opencog;

#define DEBUG

SenseSimilaritySQL::SenseSimilaritySQL(void)
{
}

SenseSimilaritySQL::~SenseSimilaritySQL()
{
}

SimpleTruthValue SenseSimilaritySQL::similarity(Handle fs, Handle ss)
{
	Handle first_sense = fs;
	Handle second_sense = ss;

	// If the parts-of-speech don't match, the similarity is zero.
	// If either one is an adjective or adverb, they're unrelated.
	// (Although we are not very confident of that!)
	std::string first_pos = get_part_of_speech(first_sense);
	std::string second_pos = get_part_of_speech(second_sense);

#if 0
	if ((0 != first_pos.compare(second_pos)) ||
	    (0 == first_pos.compare("adj")) ||
	    (0 == first_pos.compare("adv")))
	{
		SimpleTruthValue stv(0.0, 0.5);
		return stv;
	}
#endif

	printf ("Ola !!\n");


	double sim = 0.5;
	SimpleTruthValue stv((float) sim, 0.9f);
	return stv;
}

/* ============================== END OF FILE ====================== */

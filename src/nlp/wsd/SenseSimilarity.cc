/*
 * SenseSimilarity.cc
 *
 * Implements various wordnet-based sense-similarity measures.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "FollowLink.h"
#include "ForeachWord.h"
#include "SenseSimilarity.h"
#include "Node.h"
#include "SimpleTruthValue.h"

using namespace opencog;

SenseSimilarity::SenseSimilarity(void)
{
}

SenseSimilarity::~SenseSimilarity()
{
}

SimpleTruthValue SenseSimilarity::lch_similarity(Handle, Handle)
{
	SimpleTruthValue stv(0.5,1.0);

	printf ("duude sense sense \n");
	return stv;
}

/* ============================== END OF FILE ====================== */

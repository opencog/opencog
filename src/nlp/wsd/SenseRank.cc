/*
 * SenseRank.cc
 *
 * Implements the PageRank graph centrality algorithm for word-senses.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>
#include <math.h>

#include "FollowLink.h"
#include "ForeachWord.h"
#include "SenseRank.h"
#include "Node.h"
#include "SimpleTruthValue.h"

using namespace opencog;

SenseRank::SenseRank(void)
{
	damping_factor = 0.15;
}

SenseRank::~SenseRank()
{
}

void SenseRank::iterate(Handle h)
{
	foreach_parse(h, &SenseRank::rank_parse, this);
}

bool SenseRank::rank_parse(Handle)
{
	printf("Hello world\n");
	return false;
}

/* ============================== END OF FILE ====================== */

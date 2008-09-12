/*
 * SenseSimilaritySQL.h
 *
 * Fetches word-sense similarity measures from an SQL
 * database where a pre-computed set has been previously
 * stored.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SENSE_SIMILARITY_SQL_H
#define OPENCOG_SENSE_SIMILARITY_SQL_H

#include "SenseSimilarity.h"
#include "SimpleTruthValue.h"

namespace opencog {

class SenseSimilaritySQL :
	public SenseSimilarity
{

	public:
		SenseSimilaritySQL(void);
		virtual ~SenseSimilaritySQL();

		virtual SimpleTruthValue similarity(Handle, Handle);
};
}

#endif /* OPENCOG_SENSE_SIMILARITY_SQL_H */

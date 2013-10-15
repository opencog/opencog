/*
 * SenseSimilaritySQL.h
 *
 * Fetches word-sense similarity measures from an SQL database where
 * a pre-computed set has been previously stored.
 *
 * XXX In the future, this class should be eliminated, by storing  the
 * the word-sense similarities in an opencog persistent store. XXX
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_SENSE_SIMILARITY_SQL_H
#define _OPENCOG_SENSE_SIMILARITY_SQL_H

#ifdef HAVE_SQL_STORAGE

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/wsd/SenseSimilarity.h>
#include <opencog/persist/sql/odbcxx.h>

namespace opencog {

class AtomSpace;

class SenseSimilaritySQL :
	public SenseSimilarity
{
private:
    AtomSpace *as;
    ODBCConnection *db_conn;

    class Response;

public:
    SenseSimilaritySQL(AtomSpace* _as);
    virtual ~SenseSimilaritySQL();
    
    virtual SimpleTruthValuePtr similarity(Handle, Handle);
};

} // namespace opencog

#endif /* HAVE_SQL_STORAGE */
#endif // _OPENCOG_SENSE_SIMILARITY_SQL_H

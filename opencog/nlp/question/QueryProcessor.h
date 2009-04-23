/*
 * QueryProcessor.h
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_QUERY_PROCESSOR_H
#define _OPENCOG_QUERY_PROCESSOR_H

#include <opencog/atomspace/types.h>

namespace opencog
{

// forward declarations
class AtomSpace;

class QueryProcessor
{
private:
	unsigned int cnt;
	AtomSpace *atom_space;

	Handle completion_handle;
	bool check_done(Handle);

public:
	QueryProcessor(AtomSpace *);
	~QueryProcessor();
	bool process_sentence(Handle);
};

} // namespace opencog

#endif // _OPENCOG_QUERY_PROCESSOR_H

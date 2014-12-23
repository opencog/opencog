/*
 * QueryModule.h
 *
 * Load the query subsystem as a module.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_QUERY_MODULE_H
#define _OPENCOG_QUERY_MODULE_H

#include <vector>
#include <opencog/server/Module.h>
#include <opencog/query/PatternSCM.h>

namespace opencog {

class QueryModule : public Module
{
	private:
		PatternSCM* _pat;
	public:
		QueryModule(CogServer&);
		virtual ~QueryModule();
		const char * id(void);
		virtual void init(void);
};

}

#endif // _OPENCOG_QUERY_MODULE_H


/*
 * QueryModule.h
 *
 * Load the query subsystem as a module.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_QUERY_MODULE_H
#define _OPENCOG_QUERY_MODULE_H

#include <opencog/atomspace/Handle.h>
#include <opencog/server/Module.h>

namespace opencog {

class QueryModule : public Module
{
	private:
		Handle do_bindlink(Handle);
		Handle do_crisp_bindlink(Handle);
	public:
		QueryModule(CogServer&);
		virtual ~QueryModule();
		const char * id(void);
		virtual void init(void);
};

}

#endif // _OPENCOG_QUERY_MODULE_H


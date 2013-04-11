/*
 * NLPTypesModule.h
 *
 * Load the NLP atom types as a module.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_NLPTYPES_MODULE_H
#define _OPENCOG_NLPTYPES_MODULE_H

#include <opencog/server/Module.h>

namespace opencog {

class NLPTypesModule : public Module
{
   public:
      NLPTypesModule(void) {}
      virtual ~NLPTypesModule() {}
      const char * id(void);
      virtual void init(void) {}
};

}

#endif // _OPENCOG_NLPTYPES_MODULE_H


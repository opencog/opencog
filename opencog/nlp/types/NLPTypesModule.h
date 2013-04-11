/*
 * NLPTypesModule.h
 *
 * Load the NLP atom types as a module.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 * Copyright (c) 2013 Linas Vepstas <linasveptas@gmail.com>
 */

// Why does this no-op module exist? because its not really a no-op!
//
// The NLP atom types get created bythe shared object constructor
// (the init and finit called in NLPTypes.cc)  However, the shared
// library init() never seems to get called, if the loader never
// finds a reason to invoke something in the shared library! So
// the danged types don't ever get initialized!  Arghh!
//
// Work around this by creating a module; when the module is loaded,
// then the types get loaded. A bit verbose, but it works. (Actually,
// just trying to module-load the so, without having the module in it,
// works just fine. Unfortunately, it also spews errors about the .so
// not being an actual module.  So this code makes it shut up.


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


/*
 * opencog/CogServer/Module.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * Copyright (C) 2011 OpenCog Foundation
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <string>

#ifndef _OPENCOG_MODULE_H
#define _OPENCOG_MODULE_H

namespace opencog
{

/**
 * DECLARE_MODULE -- Declare a new module, called MODNAME
 * This macro implements the various routines to allow the module
 * system to work. 
 */
#define DECLARE_MODULE(MODNAME)                                       \
    /* load/unload functions for the Module interface */              \
    extern "C" const char* opencog_module_id(void) {                  \
       return "opencog::" #MODNAME;                                   \
    }                                                                 \
    extern "C" Module * opencog_module_load(CogServer *cogserver) {   \
       return new MODNAME();                                          \
    }                                                                 \
    extern "C" void opencog_module_unload(Module * m) {               \
       delete m;                                                      \
    }                                                                 \
    inline const char * MODNAME::id(void) {                           \
        return "opencog::" #MODNAME;                                  \
    }


/**
 * This class defines the base abstract class that should be extended
 * by all opencog modules.
 *
 * Since dlopen & co. provide a C API that uses runtime symbols
 * (i.e. strings) to load and unload the modules, it's not possible to
 * enforce full compliance at compile type by inheritance. The class
 * typedefs the function signatures for convenience and declares the
 * symbol names that must be used by derived modules, but the actual
 * creation and destruction functions cannot checked at compile time.
 * The only compile time check that is made is the init() method,
 * which is a late initialization entry point that modules can use to
 * initialize structures that depend on the module's constructor being
 * completed and/or the module's metadata (id, filename, function 
 * pointers, etc).
 *
 * That said, creating a proper module is fairly simple and only
 * requires 2 simple steps:
 *
 * 1. Define a class that derives from opencog::Module. It must provide
 * a static 'const char*' member which will be used to identify this
 * module.
 *
 * // DerivedModule.h
 * #include <opencog/server/Module.h>
 * class DerivedModule : public opencog::Module
 * {
 *     static const char* id = "DerivedModule"
 * }
 *
 * 2. In the class implementation, define three external C functions with
 * signatures and names matching those defined in the base Module class:
 *
 * // DerivedModule.cc
 * #include "DerivedModule.h"
 * DECLARE_MODULE(DerivedModule);
 *
 * To implement the module's functionality, you will probably want to
 * write a custom constructor and destructor and perhaps overwrite the
 * init() method (which is called by the cogserver) after the module's
 * initialization has finished and the meta-data properly set.
 */

class CogServer;

class Module
{

public:

    static const char* id_function_name(void)
    {
         static const char* s = "opencog_module_id";
         return s;
    }
    static const char* load_function_name(void)
    {
        static const char* s = "opencog_module_load";
        return s;
    }
    static const char* unload_function_name(void)
    {
        static const char* s = "opencog_module_unload";
        return s;
    }

    typedef const char* IdFunction    (void);
    typedef Module*     LoadFunction  (CogServer*);
    typedef void        UnloadFunction(Module*);

    virtual ~Module() {};
    virtual void init() = 0;

}; // class

}  // namespace

#endif // _OPENCOG_MODULE_H

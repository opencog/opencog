/**
 * NLPTypes.cc
 *
 * Atom Types used during NLP processing.
 *
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#include <opencog/server/Module.h>
#include "opencog/nlp/types/atom_types.definitions"

using namespace opencog;

// library initialization
#if defined(WIN32) && defined(_DLL)
namespace win {
#include <windows.h>
}

win::BOOL APIENTRY DllMain(win::HINSTANCE hinstDLL,  // handle to DLL module
                           win::DWORD fdwReason,     // reason for calling function
                           win::LPVOID lpvReserved)  // reserved
{
    System::setModuleHandle(hinstDLL);
    switch (fdwReason) {
        case DLL_PROCESS_ATTACH:
            #include "opencog/nlp/types/atom_types.inheritance"
            break;
        case DLL_THREAD_ATTACH:
            break;
        case DLL_THREAD_DETACH:
            break;
        case DLL_PROCESS_DETACH:
            break;
    }
    return TRUE;
}
#elif __GNUC__
static __attribute__ ((constructor)) void nlp_init(void)
{
    #include "opencog/nlp/types/atom_types.inheritance"
}

static __attribute__ ((destructor)) void nlp_fini(void)
{
}

#endif

// namespace opencog {
extern "C" {
// Calling this forces this shared-lib to load, thus calling the 
// constructor above, thus causing the atom types to be loaded into
// the atomspace.
void nlp_types_init(void)
{
	/* No-op */
}
};
// }

TRIVIAL_MODULE(NLPTypesModule)
DECLARE_MODULE(NLPTypesModule)

/**
 * PLNTypes.cc
 *
 * Atom Types used during PLN processing.
 *
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#include <opencog/server/Module.h>
#include "opencog/reasoning/pln/atom_types.definitions"

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
            #include "opencog/reasoning/pln/atom_types.inheritance"
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
static __attribute__ ((constructor)) void pln_init(void)
{
    #include "opencog/reasoning/pln/atom_types.inheritance"
}

static __attribute__ ((destructor)) void pln_fini(void)
{
}

#endif

TRIVIAL_MODULE(PLNTypesModule)
DECLARE_MODULE(PLNTypesModule)

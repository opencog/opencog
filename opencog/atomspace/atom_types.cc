/**
 * atom_types.cc
 *
 * Generic Atom Types declaration
 *
 * Copyright (c) 2009, 2014 Linas Vepstas <linasvepstas@gmail.com>
 */

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
            #include INHERITANCE_FILE
            #ifdef INHERITANCE_FILE2
            #include INHERITANCE_FILE2
            #endif
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
static __attribute__ ((constructor)) void init(void)
{
    #include INHERITANCE_FILE
    #ifdef INHERITANCE_FILE2
    #include INHERITANCE_FILE2
    #endif
}

static __attribute__ ((destructor)) void fini(void)
{
}

#endif

// namespace opencog {
extern "C" {
// Calling this forces this shared-lib to load, thus calling the 
// constructor above, thus causing the atom types to be loaded into
// the atomspace.
void INITNAME(void)
{
	/* No-op */
}
};
// }

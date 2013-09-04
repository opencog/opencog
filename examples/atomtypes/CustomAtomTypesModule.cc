/*
 * examples/modules/CustomAtomTypesModule.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include "CustomAtomTypesModule.h"

#include <time.h>

#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

#include "CustomAtomTypesTester.h"
#include "examples/atomtypes/atom_types.definitions"

using namespace opencog;

// load/unload functions for the Module interface
DECLARE_MODULE(CustomAtomTypesModule)

CustomAtomTypesModule::CustomAtomTypesModule(CogServer& cs) : Module(cs)
{
}

CustomAtomTypesModule::~CustomAtomTypesModule()
{
}

void CustomAtomTypesModule::init()
{
    logger().info("-------------------------------------------------------");
    CustomAtomTypesTester::createAtoms();
    logger().info("-------------------------------------------------------");
    CustomAtomTypesTester::dumpAtoms();
}

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
            #include "examples/atomtypes/atom_types.inheritance"
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
static __attribute__ ((constructor)) void _init(void)
{
    #include "examples/atomtypes/atom_types.inheritance"
}

static __attribute__ ((constructor)) void _fini(void)
{
}

#endif

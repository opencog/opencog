/*
 * opencog/embodiment/AGISimSim/shared/include/simcommon.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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

#ifndef SIMCOMMON_H
#define SIMCOMMON_H

#ifdef WIN32
#define HAVE_SNPRINTF
#endif

#include <math.h>
#include <stdarg.h>
#include <string>
#include <map>

#include "simconfig.h"

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>

using namespace boost;

#include "utils.h"
#include "log.h"

//------------------------------------------------------------------------------------------------------------
/// "Variable" Singletons are sometimes created with this structure
//------------------------------------------------------------------------------------------------------------
#define SINGLETON(TYPE, VAR) TYPE& VAR() { \
        static TYPE* s = new TYPE(); \
        return *s; \
    }

//------------------------------------------------------------------------------------------------------------
/** Some helper macros to define a 'bridge' design pattern.
 They're ugly, but the resulting bridges are pretty. */
/// Put this inside the bridge header definition:
//------------------------------------------------------------------------------------------------------------
#define DEFINE_BRIDGE(__iclass)   \
    protected:        \
    static __iclass* implementation; \
    public:        \
    static __iclass& Get();    \
    static __iclass& ResetBridge();  \
    protected:

//------------------------------------------------------------------------------------------------------------
/// These 3 definitions in cpp file, in this order:
//------------------------------------------------------------------------------------------------------------
#define IMPLEMENT_BRIDGE(__iclass, __par)   \
    __iclass* __iclass::implementation = NULL; \
    __iclass& __iclass::ResetBridge()    \
    {            \
        if (implementation) {      \
            delete implementation;     \
            implementation = NULL;     \
        }           \
        return Get();        \
    }            \
    __iclass& __iclass::Get()      \
    {            \
        if (!implementation)      \
        {

//------------------------------------------------------------------------------------------------------------
/** BRIDGE_PROVIDER for each implementation (__imp)
 which is selected if __condition is true.
 You can pass constructor parameters in __par or leave it empty.*/
//------------------------------------------------------------------------------------------------------------
#define BRIDGE_PROVIDER(__imp, __condition, __par)  \
    if (__condition)         \
        implementation = new __imp(__par);

#define END_BRIDGE }   \
    return *implementation; \
           }

#endif

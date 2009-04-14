/*
 * opencog/embodiment/Control/Predavese/PredaveseStdafx.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#ifndef PREDAVESE_STDAFX_H
#define PREDAVESE_STDAFX_H

// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN        // Exclude rarely-used stuff from Windows headers
#endif

#include <stdio.h>
#ifdef WIN32
#include <tchar.h>
#endif

#include <string>
#include <map>
#include <vector>
#include <set>
#include <boost/variant.hpp>

#ifdef WIN32
#include <boost/foreach.hpp>
#else
//#include "foreach.hpp"
#include <opencog/util/foreach.h>
#endif

#ifdef WIN32
#define foreach BOOST_FOREACH
#endif

#include <algorithm>

#endif

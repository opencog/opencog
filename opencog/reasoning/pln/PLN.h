/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

/** @namespace opencog::pln
 *
 * @todo Replace the numerous heap allocated TruthValue pointers throughout the
 * Rules with references instead (Particular in the rules).
 * @defgroup PLN Probabilistic Logic Networks
 */

#ifdef _PLN_H
#define _PLN_H

// What does this do, I don't know... GCC ignores it
#ifdef WIN32
#pragma warning( disable : 4786)
#pragma warning( disable : 4503)
#endif

// C includes
#include <sys/types.h>
#include <types.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Boost includes
#include <boost/smart_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>

// The fixed length of a pattern
#define PLN_CONFIG_PATTERN_LENGTH 13
#define PLN_CONFIG_FIM 1
#define PLN_CONFIG_COLLAPSE_LIST_LINKS 0

#define foreach BOOST_FOREACH
#define Btr boost::shared_ptr

#ifdef HAVE_UBIGRAPH
 #define USE_BITUBIGRAPHER
#endif

using namespace opencog;

namespace opencog {
namespace pln {
    typedef unsigned char byte;
    typedef unsigned short int word;
    typedef unsigned long int dword;
    typedef tree<Vertex> vtree;

    typedef unsigned long ulong;
    enum MetaProperty { NONE, STRENGTH, CONFIDENCE, STRENGTH_CONFIDENCE, LTI, STI };


}}

#endif // _PLN_H

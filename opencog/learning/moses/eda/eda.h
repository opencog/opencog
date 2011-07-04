/*
 * opencog/modes/eda/eda.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
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
#ifndef _EDA_EDA_H
#define _EDA_EDA_H

#include "using.h"

#include <opencog/util/tree.h>
#include <opencog/util/foreach.h>

#include <set>
#include <climits>
#include <limits>
#include <boost/static_assert.hpp>
#include <boost/variant.hpp>

//#define EDALIB_64

namespace opencog { 
namespace eda {
//storage types for packed populations
#ifdef EDALIB_64
#  if LLONG_MAX==9223372036854775807LL
typedef unsigned long long int packed_t;
#  elif LONG_MAX==9223372036854775807LL
typedef unsigned long int packed_t;
#  elif INT_MAX==9223372036854775807LL
typedef unsigned int packed_t;
#  else
#    error EDALIB_64 is defined, but cant find a 64-bit type to use
#  endif
#elif INT_MAX==2147483647
typedef unsigned int packed_t; //if EDALIB_64 is not set, just use ints
#else
#  error EDALIB_64 is not defined, so ints must be 32 bits (but they arent)
#endif

//shorthands for the number of bits in packed type
#ifdef EDALIB_64
BOOST_STATIC_ASSERT(std::numeric_limits<packed_t>::digits == 64);
const unsigned int bits_per_packed_t = 64;
#else
BOOST_STATIC_ASSERT(std::numeric_limits<packed_t>::digits == 32);
const unsigned int bits_per_packed_t = 32;
#endif

//value types accessing unpacked instances
typedef double       contin_t;
typedef int          disc_t;
typedef std::string  onto_t;
typedef tree<onto_t> onto_tree;

typedef std::vector<packed_t> instance;

} // ~namespace eda
} // ~namespace opencog

#endif

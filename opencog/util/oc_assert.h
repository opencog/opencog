/*
 * opencog/util/oc_assert.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_ASSERT_H
#define _OPENCOG_ASSERT_H

#include <opencog/util/exceptions.h>

/** \addtogroup grp_cogutil
 *  @{
 */

/** @name Assertions
 *  Macro OC_ASSERT corresponding to opencog::cassert(TRACE_INFO, ...)
 *  if IGNORE_OC_ASSERT is not defined, and ignored otherwise.
 *
 *  To disable assertions:
 *  @code
 *  #define IGNORE_OC_ASSERT
 *  @endcode
 */
///@{

#ifndef IGNORE_OC_ASSERT
#define OC_ASSERT(cond,...) \
    /* Test cond first, so that __VA_ARGS__ are NOT evaluated */ \
    /* unless cond is false! (Sometimes, evaluating the args is */ \
    /* CPU intensive, and/or requires taking locks!) */ \
    { bool test = (cond); \
    if (not test) opencog::cassert(TRACE_INFO, test, ##__VA_ARGS__); }
#else
#define OC_ASSERT(...) \
    ((void)0)
#endif

namespace opencog {

//! cassert complet with message and trace info
void cassert(const char * trace,  bool condition, const char * msg, ...);

//! cassert with a string instead of const char* and vaargs
void cassert(const char* trace, bool condition, const std::string& msg);

//! cassert without message. Just trace information
void cassert(const char * trace, bool condition);

} // namespace opencog
///@}
/** @}*/


#endif

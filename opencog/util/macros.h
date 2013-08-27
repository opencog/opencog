/*
 * opencog/util/macros.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nicu Tofan <xtnickx@users.sourceforge.net>
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

#ifndef _OPENCOG_MACROS_H
#define _OPENCOG_MACROS_H

// needed by CHECK_FREAD
#include <boost/current_function.hpp>
/** \addtogroup grp_cogutil
 *  @{
 */

//! helper to create strings at compile time
#define STRINGIFY(x) #x
//! convert the symbol to a string
#define TOSTRING(x) STRINGIFY(x)
//! indicate current file and line using (file:line) format
#define TRACE_INFO " (" __FILE__ ":" TOSTRING(__LINE__) ")"
//! supress compiler warnings about unused variables
#define OC_UNUSED(varname)	(void)varname;
//! check the return value of a fread; sets b_read local variable 
//! to false for failure; b_read must have beed declared before:
//! bool b_read = true;
#define FREAD_CK(ptr,size,count,stream) \
    b_read = b_read && (fread(ptr,size,count,stream)==(size_t)count)
//! check the b_read used for fread function and throw an exception
//! if b_read is false
#define CHECK_FREAD \
    { if ( !b_read ) throw IOException(TRACE_INFO, "%s - failed to read.", BOOST_CURRENT_FUNCTION ); }


/** @}*/
#endif // _OPENCOG_MACROS_H

/*
 * opencog/util/dorepeat.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
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

#ifndef dorepeat
/** \addtogroup grp_cogutil
 *  @{
 */

//!@{
//! helper for dorepeat(n)
#define __DOREPEAT_CONCAT_3_( a, b ) a##b
#define __DOREPEAT_CONCAT_2_( a, b ) __DOREPEAT_CONCAT_3_( a, b )
#define __DOREPEAT_CONCAT( a, b ) __DOREPEAT_CONCAT_2_( a, b )
#define __DOREPEAT_UNIQUE_NAME __DOREPEAT_CONCAT( DOREPEAT_UNIQUE_NAME_, __LINE__ )
//!@}

//! dorepeat(n) foo
/** repeats foo n times
 * maybe its mean to other people use this, but its certainly nicer that typing
 * for (int i=0;i<n;++i) when you never use i!
 */
#define dorepeat(N) \
    for (unsigned int __DOREPEAT_UNIQUE_NAME=N; __DOREPEAT_UNIQUE_NAME>0;--__DOREPEAT_UNIQUE_NAME)

/** @}*/

#endif

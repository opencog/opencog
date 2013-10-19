/*
 * opencog/atomspace/AtomSpaceDefinitions.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008-2011 OpenCog Foundation
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

/**
 * A special header file to put all definitions needed by AtomSpace's classes.
 */

#ifndef _OPENCOG_ATOMSPACE_DEFINITIONS_H_
#define _OPENCOG_ATOMSPACE_DEFINITIONS_H_
/** \addtogroup grp_atomspace
 *  @{
 */

//!@{
//! Atom flag
// #define WRITE_MUTEX             1  //BIT0
#define MARKED_FOR_REMOVAL      2  //BIT1
// #define MULTIPLE_TRUTH_VALUES   4  //BIT2
// #define FIRED_ACTIVATION        8  //BIT3
#define HYPOTETHICAL_FLAG       16 //BIT4
#define REMOVED_BY_DECAY        32 //BIT5
//!@}

/** @}*/
#endif // _OPENCOG_ATOMSPACE_DEFINITIONS_H_

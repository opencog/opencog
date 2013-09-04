/*
 * opencog/util/octime.h
 *
 * Copyright (C) 2011 OpenCog Foundation
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

#ifndef OPENCOG_UTILS_TIME_H
#define OPENCOG_UTILS_TIME_H

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

//! Initializes the reference time that will be used for getting current elapsed times
void initReferenceTime();

//! Gets the elapsed time (in milliseconds) since the reference time.
/**
 * Reference time is initialized with initReferenceTime() function.
 * The initReferenceTime() function must be called before
 * first invocation of this function.
 */
unsigned long getElapsedMillis();

/** @}*/
} // namespace opencog

#endif //  OPENCOG_UTILS_TIME_H

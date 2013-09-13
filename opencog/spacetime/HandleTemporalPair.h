/*
 * opencog/spacetime/HandleTemporalPair.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _OPENCOG_HANDLE_TEMPORAL_PAIR_H
#define _OPENCOG_HANDLE_TEMPORAL_PAIR_H

#include <opencog/atomspace/Handle.h>
#include <opencog/spacetime/Temporal.h>

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

class HandleTemporalPair
{

public:
    HandleTemporalPair();
    HandleTemporalPair(Handle, Temporal*);
    virtual ~HandleTemporalPair();

    Handle getHandle() const;
    Temporal* getTemporal() const;
    std::string toString() const;
    HandleTemporalPair clone();

private:
    Handle handle;
    Temporal* time;

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_HANDLE_TEMPORAL_PAIR_H

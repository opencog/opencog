/*
 * opencog/atomspace/Trail.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Guilherme Lamaciee
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

#ifndef _OPENCOG_TRAIL_H
#define _OPENCOG_TRAIL_H

#include <deque>
#include <exception>

#include <opencog/atomspace/types.h>
#include <opencog/util/exceptions.h>
#ifdef ZMQ_EXPERIMENT
#include "ProtocolBufferSerializer.h"
#endif

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class Trail
{
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;
#endif

    int maxSize;
    std::deque<Handle>* trail;

    void init(int, int) throw (InvalidParamException, std::bad_exception);

public:


    Trail() throw (InvalidParamException, std::bad_exception);
    Trail(int) throw (InvalidParamException, std::bad_exception);
    Trail(int, int) throw (InvalidParamException, std::bad_exception);

    ~Trail();

    bool isInTrail(Handle);

    void insert(Handle, bool = true);

    void append(Trail*);

    size_t getSize();

    void print();
    void print(FILE*);

    Handle getElement(int) throw (IndexErrorException, std::bad_exception);
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_TRAIL_H

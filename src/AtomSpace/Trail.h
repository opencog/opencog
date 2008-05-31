/*
 * src/AtomSpace/Trail.h
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

#ifndef TRAIL_H
#define TRAIL_H


#include <deque>
#include <exception>

#include "types.h"
#include "exceptions.h"

using namespace std;

class Trail
{

    int maxSize;
    deque<Handle>* trail;

    void init(int, int) throw (InvalidParamException, std::bad_exception);

public:


    Trail() throw (InvalidParamException, std::bad_exception);
    Trail(int) throw (InvalidParamException, std::bad_exception);
    Trail(int, int) throw (InvalidParamException, std::bad_exception);

    ~Trail();

    bool isInTrail(Handle);

    void insert(Handle, bool = true);

    void append(Trail*);

    int getSize();

    void print();
    void print(FILE*);

    Handle getElement(int) throw (IndexErrorException, std::bad_exception);
};

#endif

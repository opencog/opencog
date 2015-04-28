/*
 * tests/embodiment/Learning/behavior/RealTimeWorldProvider.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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
#ifndef _REAL_TIME_WORLD_PROVIDER_H_
#define _REAL_TIME_WORLD_PROVIDER_H_
#include "WorldProvider.h"
#include "PAITestUtil.h"

class RealTimeWorldProvider : public WorldProvider
{
    AtomSpace* atomSpace;
public:
    RealTimeWorldProvider(AtomSpace* _atomSpace) : atomSpace(_atomSpace) {}
    unsigned long getLatestSimWorldTimestamp() const {
        return opencog::pai::PAITestUtil::getCurrentTimestamp();
    }
    AtomSpace* getAtomSpace() const {
        return atomSpace;
    }
};

#endif // _REAL_TIME_WORLD_PROVIDER_H_


/*
 * opencog/embodiment/Learning/behavior/PAIWorldProvider.h
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

#ifndef _PAIWORLDPROVIDER_H
#define _PAIWORLDPROVIDER_H

#include "WorldProvider.h"
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

using opencog::pai::PAI;

class PAIWorldProvider : public WorldProvider
{
    PAI* pai;
public:
    PAIWorldProvider(PAI* _pai);
    unsigned long getLatestSimWorldTimestamp() const;
    AtomSpace& getAtomSpace() const;
};

#endif

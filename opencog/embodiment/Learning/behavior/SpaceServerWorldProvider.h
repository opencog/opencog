/*
 * opencog/embodiment/Learning/behavior/SpaceServerWorldProvider.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#ifndef _SPACESERVERWORLDPROVIDER_H
#define _SPECESERVERWORLDPROVIDER_H

#include "WorldProvider.h"
#include "SpaceServer.h"

//That WorldProvider is used when one would need a simple
//WorldProvider implementation, used for UTest for instance
class SpaceServerWorldProvider : public WorldProvider
{
  SpaceServer& _ss;
  unsigned long _latestSimWorldTimestamp;
public:
  SpaceServerWorldProvider(SpaceServer& ss,
			   unsigned long latestSimWorldTimestamp = 0);
  unsigned long getLatestSimWorldTimestamp() const;
  void setLatestSimWorldTimestamp(unsigned long t);
  SpaceServer& getSpaceServer() const;
};

#endif

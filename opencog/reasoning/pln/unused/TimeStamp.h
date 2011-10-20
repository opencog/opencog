/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include "Temporal.h"

using namespace opencog;

namespace opencog { namespace pln {

/**
 * This class is just an extension of Temporal class for indicating 
 * that its attribute values are actually related to a time stamp.
 */
class TimeStamp : public Temporal {

friend class TimeServer; 

public:
    TimeStamp(bool, unsigned long);
	virtual ~TimeStamp();
    unsigned long getValue();

protected:    
    TimeStamp(bool, unsigned long, unsigned long);
    
};

}} // ~namespace opencog::pln

#endif //TIMESTAMP_H

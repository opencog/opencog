/*
 * opencog/util/recent_val.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#ifndef _OPENCOG_RECENT_VAL_H
#define _OPENCOG_RECENT_VAL_H

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

//! recent_val is a value that can update which also
//! keeps a exponential decaying record of it's recent value
template<class ValueType> struct recent_val {
    //! The current value
    ValueType val;
    //! The recent record of the value
    float recent;
    //! The decay rate of the recent value
    float decay;

	//! constructor with initial value
    recent_val(ValueType x): val(x), recent((float)x), decay(0.5f) {}
	//! constructor with 0 as initial value
    recent_val(): val(0), recent(0.0f), decay(0.5f) {}
	
	//! update the current value and the decaying record
    /**
     * \param x the new value
     */
    inline void update(ValueType x) {
        val = x;
        recent = ((decay) * val) + ((1.0f - decay) * recent);
    }
};

/** @}*/
}

#endif // _OPENCOG_RECENT_VAL_H

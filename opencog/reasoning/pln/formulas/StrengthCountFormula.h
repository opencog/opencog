/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008-2009 by OpenCog Foundation
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

#ifndef _STRENGTH_COUNT_FORMULA
#define _STRENGTH_COUNT_FORMULA

#include "StrengthFormula.h"

namespace opencog { namespace pln {

// abstract class to be inherited by formula involving TV strength
class StrengthCountFormula : public StrengthFormula {

public:

    // compute TruthValue count based on the input atoms
    // TV counts
    virtual count_t computeCount(const count_seq& cs) const = 0;
};

}} // ~namespace opencog::pln
#endif

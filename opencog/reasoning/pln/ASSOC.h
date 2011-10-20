/*
 * ASSOC.h
 *
 * Copyright (C) 2009 by OpenCog Foundation
 * Written by Nil Geisweiller
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

#ifndef _PLN_ASSOC_H
#define _PLN_ASSOC_H

#include "AtomSpaceWrapper.h"
#include <opencog/atomspace/SimpleTruthValue.h>

namespace opencog { namespace pln {

// suffix to add to the concept created by CreateConceptASSOC
const std::string ASSOC_suffix = "__ASSOC__";

/**
 * CreateConceptASSOC is in charge of creating the concept AC such
 * that AC(x) = ASSOC(x, C).
 *
 * where ASSOC(F, G) = max(P(F|G) − P(F|Not G), 0)
 *
 * Here P(F|G) will be measured by SubSet G F
 *
 * This procedure is used by Intensional Inheritance.
 *
 * It operates on an AtomSpaceWrapper. If it turns out that creating ASSOC
 * is useful by other modules than PLN the code can be moved to operate
 * over AtomSpace directly.
 *
 * @param asw the AtomSpaceWrapper over which the creating takes place
 * @param c_h the pHandle of the concept, must be of type or subtype CONCEPT
 * 
 * @return the pHandle containing the concept AC, such that
 * forall x, AC(x) = ASSOC(x, c_h), the name of AC is Name__ASSOC__, where
 * Name is the name of c_h
 */
pHandle CreateConceptASSOC(AtomSpaceWrapper* asw, pHandle c_h);


/**
 * @param tv1 TruthValue representing P(F|G)
 * @param tv2 TruthValue representing P(F|Not G)
 *
 * @return SimpleTruthValue such that strength = max(tv1.s - tv2.s, 0)
 */
SimpleTruthValue ASSOC(const TruthValue& tv1, const TruthValue& tv2);

}}

#endif

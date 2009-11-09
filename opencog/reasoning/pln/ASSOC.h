/*
 * ASSOC.h
 *
 * Copyright (C) 2009 by Singularity Institute for Artificial Intelligence
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

namespace opencog { namespace pln {

/**
 * ASSOC is in charge of creating ASSOC of concepts as defined in the PLN book.
 *
 * In the PLN book ASSOC(F, G) = max(P(F|G) − P(F|NOT G), 0)
 *
 * Here P(F|G) will be measured by SubSet G F
 * 
 * and what that function will return is 
 *
 * This class is used for Intensional Inheritance.
 * It operates on an AtomSpaceWrapper.
 * If it turns out that creating ASSOC is useful by other modules than PLN
 * the code can be moved to operate over AtomSpace directly.
 *
 * @param asw the AtomSpaceWrapper over which the creating takes place
 * @param concept the pHandle of the concept, it must be of (sub)type CONCEPT
 * 
 * @return the pHandle containing the concept ASSOC_conceptName, such that
 * forall E (sub)type CONCEPT_NODE, ASSOC_conceptName(E) = ASSOC(E, concept)
 */
pHandle ASSOC(AtomSpaceWrapper* asw, pHandle concept);

}}

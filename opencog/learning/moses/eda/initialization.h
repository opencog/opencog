/*
 * opencog/learning/moses/eda/initialization.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _EDA_INITIALIZATION_H
#define _EDA_INITIALIZATION_H

#include <opencog/util/RandGen.h>

#include "../representation/field_set.h"

//various routines for initializing instances
namespace opencog { 
namespace eda {

// occam randomize a particular contin or onto field. Note that it is
// not strictly occam in the Solomonoff sense because the size is
// uniformely distributed.
void occam_randomize_contin(const field_set&, instance&,
                            field_set::contin_iterator,
                            RandGen& rng);
void occam_randomize_onto(const field_set&, instance&,
                          field_set::const_onto_iterator,
                          RandGen& rng);

//occam randomize all contin or onto fields
void occam_randomize_onto(const field_set&, instance&,
                          RandGen& rng);
void occam_randomize_contin(const field_set&, instance&,
                            RandGen& rng);

//uniformly randomize all bit or disc fields
void uniform_randomize_bits(const field_set&, instance&,
                            RandGen& rng);
void uniform_randomize_disc(const field_set&, instance&,
                            RandGen& rng);

//occam randomize all contin and onto fields, and uniformly randomize all bit
//and disc fields
void randomize(const field_set&, instance&,
               RandGen& rng);

} // ~namespace eda
} // ~namespace opencog

#endif

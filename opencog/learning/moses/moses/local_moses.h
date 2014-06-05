/*
 * opencog/learning/moses/moses/local_moses.h
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
#ifndef _MOSES_LOCAL_MOSES_H
#define _MOSES_LOCAL_MOSES_H

#include "../metapopulation/metapopulation.h"
#include "../metapopulation/deme_expander.h"
#include "moses_params.h"

namespace opencog {
namespace moses {

using namespace combo;

// A version of moses that runs only on the local host
// i.e. its not network-distributed.
void local_moses(metapopulation&,
                 deme_expander&,
                 const moses_parameters&,
                 moses_statistics&);

} // ~namespace moses
} // ~namespace opencog

#endif

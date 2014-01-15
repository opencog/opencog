/*
 * opencog/learning/pln/PLNTypes.cc
 *
 * Copyright (C) 2014 Cosmo Harrigan
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
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

#include <opencog/server/Module.h>
#include "opencog/learning/pln/atom_types.definitions"

// using namespace std;

/*
 * PLNTypes is a trivial module that implements additional atom types for
 * Probabilistic Logic Networks. The actual PLN implementation is located
 * in the opencog/python/pln folder.  
 */
__attribute__((constructor))
static void init(void)
{
    #include "opencog/learning/pln/atom_types.inheritance"
}

// __attribute__((constructor))
// void fini(void)
// {
// }

using namespace opencog;
TRIVIAL_MODULE(PLNTypesModule)
DECLARE_MODULE(PLNTypesModule)

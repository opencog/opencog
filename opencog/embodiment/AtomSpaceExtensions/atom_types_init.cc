/*
 * opencog/embodiment/AtomSpaceExtensions/atom_types_init.cc
 *
 * Copyright (C) 2014 Linas Vepstas
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

#include "opencog/spacetime/atom_types.definitions"
#include "opencog/embodiment/AtomSpaceExtensions/atom_types.definitions"

#define INHERITANCE_FILE "opencog/spacetime/atom_types.inheritance"
#define INHERITANCE_FILE2 "opencog/embodiment/AtomSpaceExtensions/atom_types.inheritance"
#define INITNAME embodiment_types_init

#include <opencog/atomspace/atom_types.cc>

using namespace opencog;
TRIVIAL_MODULE(EmbodimentTypesModule)
DECLARE_MODULE(EmbodimentTypesModule)


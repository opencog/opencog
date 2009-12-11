/*
 * opencog/atomspace/TLB.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#include "TLB.h"

#include <opencog/util/platform.h>

#ifdef USE_TLB_MAP

using namespace opencog;

// XXX As of March 2009, non-real atoms are now obsolete!
// Remove this stuff after the dust clears.
// Low-lying values are reserved for "non-real" atoms. Real atom start after the
// last definable type (opencog::NOTYPE) opencog::NOTYPE is defined as
// ((1 << (8 * sizeof(opencog::Type))) - 1), which is 65535 when Type is "unsigned
// short int"
// UUID TLB::brk_uuid = (1 << (8 * sizeof(opencog::Type)));
UUID TLB::brk_uuid = 1;

boost::unordered_map<Handle, const Atom*, boost::hash<opencog::Handle> > TLB::handle_map;

#endif

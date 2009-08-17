/*
 * src/util/misc-test.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_MISC_TEST_H
#define _OPENCOG_MISC_TEST_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/util/misc.h>

namespace opencog
{

/**
 * Handy dandy utilty used by some test cases
 */
Handle addAtom(AtomSpace& as, tree<Vertex>& a, const TruthValue& tvn);

} // namespace opencog

#endif // _OPENCOG_MISC_TEST_H

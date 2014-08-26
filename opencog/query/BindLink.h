/*
 * BindLink.h
 *
 * Copyright (C) 2014 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_BINDLINK_H
#define _OPENCOG_BINDLINK_H

#include <opencog/atomspace/AtomSpace.h>

namespace opencog {

Handle bindlink(AtomSpace*, Handle);
Handle single_bindlink (AtomSpace*, Handle);
Handle crisp_logic_bindlink(AtomSpace*, Handle);
Handle pln_bindlink(AtomSpace*, Handle);

} // namespace opencog

#endif // _OPENCOG_BINDLINK_H

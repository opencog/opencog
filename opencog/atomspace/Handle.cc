/*
 * opencog/atomspace/Handle.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2013 Linas Vepstas <linas@linas.org>
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

#include <climits>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomTable.h>

using namespace opencog;

const Handle Handle::UNDEFINED(ULONG_MAX);

Handle::Handle(AtomPtr atom) : _uuid(atom->_uuid), _ptr(atom) {}

Handle& Handle::operator=(const AtomPtr& a)
{
    this->_uuid = a->_uuid;
    this->_ptr = a;
    return *this;
}

// ===================================================
// Handle resolutino stuff.

void Handle::set_resolver(const AtomTable* tab)
{
    _resolver = tab;
}

void Handle::clear_resolver(const AtomTable* tab)
{
    _resolver = NULL;
}

const AtomTable* Handle::_resolver = NULL;

Atom* Handle::resolve()
{
    AtomPtr a(_resolver->getHandle(*this)._ptr);
    _ptr.swap(a);
    return _ptr.get();
}

Atom* Handle::cresolve() const
{
    return _resolver->getHandle(*this)._ptr.get();
}

AtomPtr Handle::resolve_ptr()
{
    AtomPtr a(_resolver->getHandle(*this)._ptr);
    _ptr.swap(a);
    return _ptr;
}

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
// Handle resolution stuff.

// Its a vector, not a set, because its priority ranked.
std::vector<const AtomTable*> Handle::_resolver;

void Handle::set_resolver(const AtomTable* tab)
{
    _resolver.push_back(tab);
}

void Handle::clear_resolver(const AtomTable* tab)
{
    auto end = _resolver.end();
    for (auto it = _resolver.begin(); it != end; it++)
    {
        if (*it == tab)
        {
            _resolver.erase(it);
            break;
        }
    }
}

// Search several atomspaces, in order.  First one to come up with
// the atom wins.  Seems to work, for now.
inline AtomPtr Handle::do_res(const Handle* hp)
{
    auto end = _resolver.end();
    for (auto it = _resolver.begin(); it != end; it++)
    {
        const AtomTable* at = *it;
        AtomPtr a(at->getHandle((Handle&)(*hp))._ptr);
        if (a.get()) return a;
    }
    return NULL;
}

Atom* Handle::resolve()
{
    AtomPtr a(do_res(this));
    _ptr.swap(a);
    return _ptr.get();
}

Atom* Handle::cresolve() const
{
    return do_res(this).get();
}

AtomPtr Handle::resolve_ptr()
{
    AtomPtr a(do_res(this));
    _ptr.swap(a);
    return _ptr;
}

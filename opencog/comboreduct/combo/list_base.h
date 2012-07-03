/*
 * opencog/comboreduct/combo/list_base.h
 *
 * Copyright (C) 2002-2008, 2012 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller, Linas Vepstas
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
#ifndef _COMBO_LIST_BASE_H
#define _COMBO_LIST_BASE_H

#include <memory>
#include <ostream>
#include <boost/operators.hpp>

namespace opencog { namespace combo {

#ifdef DEBUG
#define DBG(X) X
#else
#define DBG(X)
#endif

/// Base class for list_t, which is really just a combo tree. :-)
/// We use this base class, together with the list_ptr below, to avoid
/// a compile-time circular definition of list_t.
class list_base
    : boost::less_than_comparable<list_base>, // generate >, <= and >= given <
      boost::equality_comparable<list_base>   // generate != given ==
{
public:
    list_base() {}

    virtual ~list_base() {}

    // XXX TODO implement me as needed!
    bool operator==(const list_base& m) const
    {
        OC_ASSERT(false, "list_base equals NOT YET IMPLEMENTED!");
        return true;
    }

    // XXX TODO implement me as needed!
    /// Lexicographic ordering on trees ... ??? does this work?
    virtual bool operator<(const list_base& m) const
    {
        OC_ASSERT(false, "list_base comparison NOT YET IMPLEMENTED!");
        return true;
    }
protected:
    friend class list_ptr;
    virtual list_base* copy() const = 0;
};


/// Wrapper around auto_ptr<list_base>.
/// We need to provide this as a class, as otherwise, the boost variant
/// used to define vertex chokes if we simply typedef this.  And, of
/// course, we need to use auto_ptr to avoid mem leaks.
class list_ptr
    : boost::less_than_comparable<list_ptr>, // generate >, <= and >= given <
      boost::equality_comparable<list_ptr>   // generate != given ==
{
public:
    list_ptr(const list_ptr &b) :
        _base(b._base->copy())
    {
        DBG(std::cout<<"list_ptr::copy ctor that="<<((void *) b._base.get())<<" copy="<< ((void *) _base.get())<<std::endl);
    }

    list_ptr(list_base *b) :
        _base(b)
    {
        DBG(std::cout<<"list_ptr::ctor ptr="<<((void*)b)<<std::endl);
    }

    virtual ~list_ptr()
    {
        DBG(std::cout<<"list_ptr::dtor() ptr="<<((void *) _base.get())<<std::endl);
    }

    list_base* get() const
    {
        return _base.get();
    }

    // XXX TODO Perhaps if we are more clever with the template, we
    // could make this class behave as if it were list_t. i.e. we
    // want to somehow return something like const list_t& here,
    // instead of a pointer. .. !?
    template<class LIST>
    LIST* getp() const
    {
        return dynamic_cast<LIST*>(get());
    }

    // The boost vertex variant seems to need this to compile, but it
    // never seems to be called at runtime !?
    list_ptr& operator=(const list_ptr& other)
    {
        OC_ASSERT(false, "list_ptr operator= not yet implemented!");
        return *this;
    }

    bool operator==(const list_ptr& m) const
    {
        return *(_base) == *(m._base);
    }

    /// Lexicographic ordering on trees ... ??? does this work?
    virtual bool operator<(const list_ptr& m) const
    {
        OC_ASSERT(false, "list_ptr comparison NOT YET IMPLEMENTED!");
        return true;
    }

private:
    std::auto_ptr<list_base>  _base;
};


} // ~namespace combo
} // ~namespace opencog

#endif


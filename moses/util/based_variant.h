/*
 * opencog/util/based_variant.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#ifndef _OPENCOG_BASED_VARIANT_H
#define _OPENCOG_BASED_VARIANT_H

#include <boost/variant.hpp>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

namespace detail {
template<typename Base>
struct based_variant_visitor : public boost::static_visitor<Base*> {
    template<typename T>
    Base* operator()(T& t) const {
        return &t;
    }
};
template<typename Base>
struct const_based_variant_visitor : public boost::static_visitor<const Base*> {
    template<typename T>
    const Base* operator()(const T& t) const {
        return &t;
    }
};
} //~namespace detail

template<typename Variant, typename Base>
struct based_variant : public Variant {
    template<typename T>
    based_variant(const T& v) : Variant(v) { }
    based_variant() { }

    Base* operator->() {
        return boost::apply_visitor(detail::based_variant_visitor<Base>(), *this);
    }
    const Base* operator->() const {
        return boost::apply_visitor(detail::const_based_variant_visitor<Base>(), *this);
    }

    /*operator Base&() {
      return *boost::apply_visitor(detail::based_variant_visitor<Base>(),*this);
    }
    operator const Base&() const {
      return *boost::apply_visitor(detail::const_based_variant_visitor<Base>(),
      *this);
      }*/
};

/** @}*/
} //~namespace opencog

#endif // _OPENCOG_BASED_VARIANT_H

#ifndef _UTIL_BASED_VARIANT_H
#define _UTIL_BASED_VARIANT_H

#include <boost/variant.hpp>

namespace Util {
  namespace detail {
    template<typename Base>
    struct based_variant_visitor : public boost::static_visitor<Base*> {
      template<typename T>
      Base* operator()(T& t) const { return &t; }
    };
    template<typename Base>    
    struct const_based_variant_visitor : public boost::static_visitor<const Base*> {
      template<typename T>
      const Base* operator()(const T& t) const { return &t; }
    };
  } //~namespace detail

  template<typename Variant,typename Base>
  struct based_variant : public Variant {
    template<typename T>
    based_variant(const T& v) : Variant(v) { }
    based_variant() { }

    Base* operator->() {
      return boost::apply_visitor(detail::based_variant_visitor<Base>(),*this); 
    }
    const Base* operator->() const {
      return boost::apply_visitor(detail::const_based_variant_visitor<Base>(),*this);
    }

    /*operator Base&() { 
      return *boost::apply_visitor(detail::based_variant_visitor<Base>(),*this); 
    }
    operator const Base&() const { 
      return *boost::apply_visitor(detail::const_based_variant_visitor<Base>(),
      *this);
      }*/
  };

} //~namespace Util

#endif

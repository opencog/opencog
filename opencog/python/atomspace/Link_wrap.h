#ifndef _OPENCOG_LINK_WRAP_H
#define _OPENCOG_LINK_WRAP_H

#include "Link.h"
#include <boost/python.hpp>
using namespace opencog;
using namespace boost::python;

/** Exposes the Link class. */
void init_Link_py();

/** A class wrapper of the Link class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct LinkWrap : Link, wrapper<Link>
{
    LinkWrap(Type, const std::vector<Handle>&, const TruthValue&);
    LinkWrap(Type, Handle&, const TruthValue&);
    LinkWrap(Type, Handle&, Handle&, const TruthValue&);
    LinkWrap(Type, Handle&, Handle&, Handle&, const TruthValue&);
    LinkWrap(Type, Handle&, Handle&, Handle&, Handle&, const TruthValue&);
    LinkWrap(const Link&);

    // Non-pure virtual functions.

    bool operator==(const Atom&) const;
    bool default_operator_equal_equal(const Atom&) const;
    bool operator!=(const Atom&) const;
    bool default_operator_not_equal(const Atom&) const;
    size_t hashCode(void) const;
    size_t default_hashCode(void) const;
};

#endif

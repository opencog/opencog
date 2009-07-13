#ifndef _OPENCOG_ATOM_WRAP_H
#define _OPENCOG_ATOM_WRAP_H

#include "Atom.h"
#include <boost/python/wrapper.hpp>

using namespace opencog;
using namespace boost::python;

/** Exposes the Atom class. */
void init_Atom_py();

/** A class wrapper of the Atom class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct AtomWrap : Atom, wrapper<Atom>
{
#ifndef PUT_OUTGOING_SET_IN_LINKS
    AtomWrap(Type, const std::vector<Handle>&, const TruthValue&);
#else
    AtomWrap(Type, const TruthValue&);
#endif

    // Pure virtual functions.

    std::string toString(void) const;
    std::string toShortString(void) const;
    bool operator==(const Atom&) const;
    bool operator!=(const Atom&) const;
    size_t hashCode(void) const;

    // Non-pure virtual functions.
};

#endif // _OPENCOG_ATOM_WRAP_H

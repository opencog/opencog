#ifndef _OPENCOG_ATOMSPACE_WRAP_H
#define _OPENCOG_ATOMSPACE_WRAP_H

#include <boost/python/wrapper.hpp>

#include <opencog/atomspace/AtomSpace.h>

using namespace boost::python;
using namespace opencog;

/** Exposes the AtomSpace class. */
void init_AtomSpace_py();

/** A class wrapper of the AtomSpace class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct AtomSpaceWrap: AtomSpace, wrapper<AtomSpace>
{
    // Non-pure virtual functions.

};

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
//Handle (*addLinkx1)(Type, const HandleSeq&, const TruthValue&) = &AtomSpace::addLink;
//Handle (*addLinkx2)(Type, Handle, const TruthValue&) = &AtomSpace::addLink;

#endif // _OPENCOG_ATOMSPACE_WRAP_H

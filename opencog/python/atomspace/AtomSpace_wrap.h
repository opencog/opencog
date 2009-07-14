#ifndef _OPENCOG_ATOMSPACE_WRAP_H
#define _OPENCOG_ATOMSPACE_WRAP_H

#include <boost/python/wrapper.hpp>

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/TruthValue.h>

using namespace opencog;
using namespace boost::python;

/** Exposes the AtomSpace class. */
void init_AtomSpace_py();

/** A class wrapper of the AtomSpace class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct AtomSpaceWrap: AtomSpace, wrapper<AtomSpace>
{
    Handle addLinkx1(Type t, const HandleSeq& outgoing, const TruthValue& tvn) {
        return addLink(t, outgoing, tvn);
    }
    Handle addLinkx2(Type t, const Handle h, const TruthValue& tvn) {
        return addLink(t, h, tvn);
    }
};

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
/*Handle (*addLinkx1)(Type t, const HandleSeq& outgoing, const TruthValue& tvn) = &AtomSpace::addLink;
Handle (*addLinkx2)(Type t, Handle h, const TruthValue& tvn) = &AtomSpace::addLink;*/

#endif // _OPENCOG_ATOMSPACE_WRAP_H

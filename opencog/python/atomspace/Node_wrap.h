#ifndef _OPENCOG_NODE_WRAP_H
#define _OPENCOG_NODE_WRAP_H

#include "Node.h"
#include <boost/python/wrapper.hpp>
using namespace opencog;
using namespace boost::python;

/** Exposes the Node class. */
void init_Node_py();

/** A class wrapper of the Node class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct NodeWrap : Node, wrapper<Node>
{
    NodeWrap(Type, const std::string&, const TruthValue&);
    NodeWrap(const Node&);

    // Non-pure virtual functions.

    bool operator==(const Atom&) const;
    bool default_operator_equal_equal(const Atom&) const;
    bool operator!=(const Atom&) const;
    bool default_operator_not_equal(const Atom&) const;
    size_t hashCode(void) const;
    size_t default_hashCode(void) const;
};

#endif

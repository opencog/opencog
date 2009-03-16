/**
 * ElementaryBehaviorDescription.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Aug 22 12:57:29 BRT 2007
 */

#ifndef ELEMENTARYBEHAVIORDESCRIPTION_H
#define ELEMENTARYBEHAVIORDESCRIPTION_H

#include <opencog/atomspace/Temporal.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/TLB.h>
#include "util/StringManipulator.h"

using namespace opencog;

namespace behavior {

class ElementaryBehaviorDescription {

    public:

        Handle handle;
        Temporal temporal;

	ElementaryBehaviorDescription() : temporal(0) {}
        ElementaryBehaviorDescription(Handle h, const Temporal& t): handle(h), temporal(t) {}

	std::string toString() const {
	  std::string str = std::string("{") +
	    (handle==Handle::UNDEFINED? std::string("Handle::UNDEFINED"):
	     opencog::toString(handle))
	    + std::string(",") + temporal.toString() + std::string("}");
	  return str;
	}

}; // class
}  // namespace

#endif

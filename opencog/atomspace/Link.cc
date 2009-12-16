/*
 * opencog/atomspace/Link.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
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

#include "Link.h"

#include <stdio.h>

#include <opencog/util/platform.h>

#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/StatisticsMonitor.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/Trail.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

using namespace opencog;

#ifndef PUT_OUTGOING_SET_IN_LINKS
void Link::init(void)
#else
void Link::init(const std::vector<Handle>& outgoingVector)
#endif
	throw (InvalidParamException)
{
    if (!classserver().isA(type, LINK)) {
        throw InvalidParamException(TRACE_INFO,
             "Link -  invalid link type: %d", type);
    }
    trail = new Trail();
#ifdef PUT_OUTGOING_SET_IN_LINKS
    setOutgoingSet(outgoingVector);
#endif
}

Link::~Link() throw ()
{
    //printf("Link::~Link() begin\n");
//    fprintf(stdout, "Deleting link:\n%s\n", this->toString().c_str());
//    fflush(stdout);
    delete trail;
    //printf("Link::~Link() end\n");
}

void Link::setTrail(Trail* t)
{
    if (trail) {
        delete(trail);
    }
    trail = t;
}

Trail* Link::getTrail(void)
{
    return trail;
}

std::string Link::toShortString(void) const
{
    std::string answer;
#define BUFSZ 1024
    char buf[BUFSZ];

    snprintf(buf, BUFSZ, "[%s %s", classserver().getTypeName(type).c_str(),
             (getFlag(HYPOTETHICAL_FLAG) ? "h " : ""));
    answer += buf;
    // Here the targets string is made. If a target is a node, its name is
    // concatenated. If it's a link, all its properties are concatenated.
    answer += "<";
    for (int i = 0; i < getArity(); i++) {
        if (i > 0) answer += ",";
        answer += classserver().isA(TLB::getAtom(outgoing[i])->getType(), NODE) ?
                  ((Node*) TLB::getAtom(outgoing[i]))->getName() :
                  ((Link*) TLB::getAtom(outgoing[i]))->toShortString();
    }
    answer += ">";
    float mean = this->getTruthValue().getMean();
    float count = this->getTruthValue().getCount();
    snprintf(buf, BUFSZ, " %f %f", mean, count);
    answer += buf;
    answer += "]";
    return answer;
}

std::string Link::toString(void) const
{
    std::string answer;
    char buf[BUFSZ];

    snprintf(buf, BUFSZ, "link[%s sti:(%d,%d) tv:(%s) ",
             classserver().getTypeName(type).c_str(),
             (int)getAttentionValue().getSTI(),
             (int)getAttentionValue().getLTI(),
             getTruthValue().toString().c_str());
    answer += buf;
    // Here the targets string is made. If a target is a node, its name is
    // concatenated. If it's a link, all its properties are concatenated.
    answer += "<";
    for (int i = 0; i < getArity(); i++) {
        if (i > 0) answer += ",";
        // Type t = TLB::getAtom(outgoing[i])->getType();
        //logger().fine("toString() => type of outgoing[%d] = %d", i, t);
        Handle h = outgoing[i];
        if (TLB::isValidHandle(h)) {
            Atom *a = TLB::getAtom(h);
            Node *nnn = dynamic_cast<Node *>(a);
            if (nnn) {
                snprintf(buf, BUFSZ, "[%s ", classserver().getTypeName(a->getType()).c_str());
                answer += buf;
                if (nnn->getName() == "")
                    answer += "#" + h;
                else
                    answer += nnn->getName();
                answer += "]";
            } else {
                Link *lll = dynamic_cast<Link *>(a);
                answer += lll->toString();
            }
        } else {
            logger().error("Link::toString() => invalid handle %lu in position %d of ougoing set!",
                           h.value(), i);
            answer += "INVALID_HANDLE!";
        }
    }
    answer += ">]";
    return answer;
}

float Link::getWeight(void)
{
    return getTruthValue().toFloat();
}

bool Link::isSource(Handle handle) throw (InvalidParamException)
{
    // On ordered links, only the first position in the outgoing set is a source
    // of this link. So, if the handle given is equal to the first position,
    // true is returned.
    if (classserver().isA(type, ORDERED_LINK)) {
        return getArity() > 0 && outgoing[0] == handle;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // if the links is unordered, the outgoing set is scanned, and the
        // method returns true if any position is equal to the handle given.
        for (int i = 0; i < getArity(); i++) {
            if (outgoing[i] == handle) {
                return true;
            }
        }
        return false;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isSource(Handle) unknown link type %d", type);
    }
}

bool Link::isSource(int i) throw (IndexErrorException, InvalidParamException)
{
    // tests if the int given is valid.
    if ((i > getArity()) || (i < 0)) {
        throw IndexErrorException(TRACE_INFO, "Link::isSource(int) invalid index argument");
    }

    // on ordered links, only the first position in the outgoing set is a source
    // of this link. So, if the int passed is 0, true is returned.
    if (classserver().isA(type, ORDERED_LINK)) {
        return i == 0;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // on unordered links, the only thing that matter is if the int passed
        // is valid (if it is within 0..arity).
        return true;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isSource(int) unknown link type %d", type);
    }
}

bool Link::isTarget(Handle handle) throw (InvalidParamException)
{
    // on ordered links, the first position of the outgoing set defines the
    // source of the link. The other positions are targets. So, it scans the
    // outgoing set from the second position searching for the given handle. If
    // it is found, true is returned.
    if (classserver().isA(type, ORDERED_LINK)) {
        for (int i = 1; i < getArity(); i++) {
            if (outgoing[i] == handle) {
                return true;
            }
        }
        return false;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // if the links is unordered, all the outgoing set is scanned.
        for (int i = 0; i < getArity(); i++) {
            if (outgoing[i] == handle) {
                return true;
            }
        }
        return false;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isTarget(Handle) unknown link type %d", type);
    }
}

bool Link::isTarget(int i) throw (IndexErrorException, InvalidParamException)
{
    // tests if the int given is valid.
    if ((i > getArity()) || (i < 0)) {
        throw IndexErrorException(TRACE_INFO, "Link::istarget(int) invalid index argument");
    }

    // on ordered links, the first position of the outgoing set defines the
    // source of the link. The other positions are targets.
    if (classserver().isA(type, ORDERED_LINK)) {
        return i != 0;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // on unorderd links, the only thing that matter is if the position is
        // valid.
        return true;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isTarget(int) unkown link type");
    }
}

bool Link::operator==(const Atom& other) const
{
    if (getType() != other.getType()) return false;
    const Link& olink = dynamic_cast<const Link&>(other);
    if (getArity() != olink.getArity()) return false;
    for (unsigned int i = 0; i < getArity(); i++)
        if (outgoing[i] != olink.outgoing[i]) return false;
    return true;
}

bool Link::operator!=(const Atom& other) const
{
    return !(*this == other);
}

size_t Link::hashCode(void) const
{
    size_t result = getType() + (getArity() << 8);
    for (unsigned int i = 0; i < getArity(); i++)
        result = result  ^ (((long) outgoing[i].value()) << i);
    return result;
}

#ifdef PUT_OUTGOING_SET_IN_LINKS
class HandleComparison
{
    public:
        bool operator()(const Handle& h1, const Handle& h2) const {
            return (Handle::compare(h1, h2) < 0);
        }
};

void Link::setOutgoingSet(const std::vector<Handle>& outgoingVector)
   throw (RuntimeException)
{
    //printf("Atom::setOutgoingSet\n");
    if (atomTable != NULL) {
        throw RuntimeException(TRACE_INFO, 
           "Cannot change the OutgoingSet of an atom already "
           "inserted into an AtomTable\n");
    }
#ifdef PERFORM_INVALID_HANDLE_CHECKS
    // Make sure that garbage is not being passed in.
    // We'd like to perform a test for valid values here, but it seems
    // the NMXmlParser code intentionally adds Handle::UNDEFINED to link nodes,
    // which it hopefully repairs later on ...
    for (size_t i = 0; i < outgoingVector.size(); i++) {
        if (TLB::isInvalidHandle(outgoingVector[i])) {
            throw RuntimeException(TRACE_INFO, "setOutgoingSet was passed invalid handles\n");
        }
    }
#endif
    outgoing = outgoingVector;
    // if the link is unordered, it will be normalized by sorting the elements in the outgoing list.
    if (classserver().isA(type, UNORDERED_LINK)) {
        std::sort(outgoing.begin(), outgoing.end(), HandleComparison());
    }
}

void Link::addOutgoingAtom(Handle h)
{
#ifdef PERFORM_INVALID_HANDLE_CHECKS
    // We'd like to perform a test for valid values here, but it seems
    // the NMXmlParser code intentionally adds Handle::UNDEFINED to link nodes,
    // which it hopefully repairs later on ...
    if (TLB::isInvalidHandle(h))
        throw RuntimeException(TRACE_INFO, "addOutgoingAtom was passed invalid handles\n");
#endif
    outgoing.push_back(h);
}

#endif /* PUT_OUTGOING_SET_IN_LINKS */


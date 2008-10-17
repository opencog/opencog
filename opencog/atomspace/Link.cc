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
#include <opencog/atomspace/CoreUtils.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/StatisticsMonitor.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/Trail.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

using namespace opencog;

void Link::init(void) throw (InvalidParamException)
{
    if (!ClassServer::isAssignableFrom(LINK, type)) {
#ifndef USE_STD_VECTOR_FOR_OUTGOING
        if (outgoing) free(outgoing);
#endif
        throw InvalidParamException(TRACE_INFO, "Link -  invalid link type: %d", type);
    }
    trail = new Trail();
}

Link::Link(Type type, const std::vector<Handle>& outgoingVector, const TruthValue& tv)
        : Atom(type, outgoingVector, tv)
{
    init();
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

    snprintf(buf, BUFSZ, "[%d %s", type, (getFlag(HYPOTETHICAL_FLAG) ? "h " : ""));
    answer += buf;
    // Here the targets string is made. If a target is a node, its name is
    // concatenated. If it's a link, all its properties are concatenated.
    answer += "<";
    for (int i = 0; i < getArity(); i++) {
        if (i > 0) answer += ",";
        answer += ClassServer::isAssignableFrom(NODE, TLB::getAtom(outgoing[i])->getType()) ?
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

    snprintf(buf, BUFSZ, "link[%d sti:(%d,%d) tv:(%f,%f) ", type,
             (int)getAttentionValue().getSTI(),
             (int)getAttentionValue().getLTI(),
             getTruthValue().getMean(),
             getTruthValue().getConfidence());
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
            Atom *a = TLB::getAtom(outgoing[i]);
            Node *nnn = dynamic_cast<Node *>(a);
            if (nnn) {
                snprintf(buf, BUFSZ, "[%d ", a->getType());
                answer += buf;
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
    if (ClassServer::isAssignableFrom(ORDERED_LINK, type)) {
        return getArity() > 0 && outgoing[0] == handle;
    } else if (ClassServer::isAssignableFrom(UNORDERED_LINK, type)) {
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
    if (ClassServer::isAssignableFrom(ORDERED_LINK, type)) {
        return i == 0;
    } else if (ClassServer::isAssignableFrom(UNORDERED_LINK, type)) {
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
    if (ClassServer::isAssignableFrom(ORDERED_LINK, type)) {
        for (int i = 1; i < getArity(); i++) {
            if (outgoing[i] == handle) {
                return true;
            }
        }
        return false;
    } else if (ClassServer::isAssignableFrom(UNORDERED_LINK, type)) {
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
    if (ClassServer::isAssignableFrom(ORDERED_LINK, type)) {
        return i != 0;
    } else if (ClassServer::isAssignableFrom(UNORDERED_LINK, type)) {
        // on unorderd links, the only thing that matter is if the position is
        // valid.
        return true;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isTarget(int) unkown link type");
    }
}

bool Link::equals(const Atom* other) const
{
    if (type != other->getType()) return false;

    const Link* olink = dynamic_cast<const Link*>(other);
    if (getArity() != olink->getArity()) return false;

    for (int i = 0; i < getArity(); i++) {
        if (outgoing[i] != olink->outgoing[i]) return false;
    }

    return true;
}

int Link::hashCode(void) const
{
    long result = type + (getArity() << 8);

    for (int i = 0; i < getArity(); i++) {
        result = result  ^ (((long) outgoing[i].value()) << i);
    }
    return (int) result;
}


/*
 * opencog/persist/zmq/events/AtomSpacePublisherModule.cc
 *
 * Copyright (C) 2013 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
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

#include "AtomSpacePublisherModule.h"

#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>
#include <opencog/util/Config.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include "opencog/util/zhelpers.hpp"
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

using namespace std;
using namespace opencog;

DECLARE_MODULE(AtomSpacePublisherModule)

AtomSpacePublisherModule::AtomSpacePublisherModule(CogServer& cs) : Module(cs)
{
    logger().info("[AtomSpacePublisherModule] constructor");
    this->as = &cs.getAtomSpace();
    addAtomConnection = as->addAtomSignal(boost::bind(&AtomSpacePublisherModule::handleAddSignal, this, _1));
    removeAtomConnection = as->removeAtomSignal(boost::bind(&AtomSpacePublisherModule::atomRemoveSignal, this, _1));
    TVChangedConnection = as->TVChangedSignal(boost::bind(&AtomSpacePublisherModule::TVChangedSignal, this, _1, _2, _3));
    AVChangedConnection = as->AVChangedSignal(boost::bind(&AtomSpacePublisherModule::AVChangedSignal, this, _1, _2, _3));
}

void AtomSpacePublisherModule::init(void)
{
    logger().info("Initializing AtomSpacePublisherModule.");
    InitZeroMQ();
}

void AtomSpacePublisherModule::run()
{
}

AtomSpacePublisherModule::~AtomSpacePublisherModule()
{
    logger().info("Terminating AtomSpacePublisherModule.");
    publisher->close();
    delete publisher;
    delete context;
}

void AtomSpacePublisherModule::InitZeroMQ()
{
    //  Prepare the context and publisher
    context = new zmq::context_t(1);
    publisher = new zmq::socket_t(*context, ZMQ_PUB);

    std::string zmq_event_port = config().get("ZMQ_EVENT_PORT");
    bool zmq_use_public_ip = config().get_bool("ZMQ_EVENT_USE_PUBLIC_IP");

    std::string zmq_ip;
    if (zmq_use_public_ip)
        zmq_ip = "0.0.0.0";
    else
        zmq_ip = "*";

    const char * zmq_address = ("tcp://" + zmq_ip + ":" + zmq_event_port).c_str();
    publisher->bind(zmq_address);
}

void AtomSpacePublisherModule::handleAddSignal(Handle h)
{
    std::string payload = atomToJSON(h);
    s_sendmore (*publisher, "add");
    s_send (*publisher, payload);
}

void AtomSpacePublisherModule::atomRemoveSignal(AtomPtr atom)
{
    std::string payload = atomToJSON(atom->getHandle());
    s_sendmore (*publisher, "remove");
    s_send (*publisher, payload);
}

void AtomSpacePublisherModule::AVChangedSignal(const Handle& h, const AttentionValuePtr& av_old, const AttentionValuePtr& av_new)
{      
    std::string payload = atomToJSON(h);
    s_sendmore (*publisher, "avchanged");
    s_send (*publisher, payload);
}

void AtomSpacePublisherModule::TVChangedSignal(const Handle& h, const TruthValuePtr& tv_old, const TruthValuePtr& tv_new)
{
    std::string payload = atomToJSON(h);
    s_sendmore (*publisher, "tvchanged");
    s_send (*publisher, payload);
}

std::string AtomSpacePublisherModule::atomToJSON(Handle h)
{
    // Type
    Type type = as->getType(h);
    std::string typeNameString = classserver().getTypeName(type);  //.c_str()

    // Name
    std::string nameString = as->getName(h);

    // Handle
    std::string handleString = std::to_string(h.value());

    // AttentionValue
    AttentionValuePtr av = as->getAV(h);
    std::string stiString = std::to_string(av->getSTI());
    std::string ltiString = std::to_string(av->getLTI());
    std::string vltiString = av->getVLTI() != 0 ? "true" : "false";

    // TruthValue
    TruthValuePtr tvp = as->getTV(h);

    ptree ptTV;

    //ptTV = tvToPtree(tvp);

    switch (tvp->getType())
    {
        case SIMPLE_TRUTH_VALUE:
        {
            ptTV.put("type", "simple");
            ptTV.put("details.strength", tvp->getMean());
            ptTV.put("details.count", tvp->getCount());
            ptTV.put("details.confidence", tvp->getConfidence());
            break;
        }

        case COUNT_TRUTH_VALUE:
        {
            ptTV.put("type", "count");
            ptTV.put("details.strength", tvp->getMean());
            ptTV.put("details.count", tvp->getCount());
            ptTV.put("details.confidence", tvp->getConfidence());
            break;
        }

        case INDEFINITE_TRUTH_VALUE:
        {
            IndefiniteTruthValuePtr itv = IndefiniteTVCast(tvp);
            ptTV.put("type", "indefinite");
            ptTV.put("details.strength", itv->getMean());
            ptTV.put("details.L", itv->getL());
            ptTV.put("details.U", itv->getU());
            ptTV.put("details.confidence", itv->getConfidenceLevel());
            ptTV.put("details.diff", itv->getDiff());
            ptTV.put("details.symmetric", itv->isSymmetric());
            break;
        }

        case COMPOSITE_TRUTH_VALUE:
        {
/*
            // TODO: CompositeTruthValue
            CompositeTruthValuePtr ctv = CompositeTVCast(tvp);
            ptTV.put("type", "composite");
            // TODO: call tvToPtree on tvp->getPrimaryTV()
            ptTV.put("details.primary", "");
            foreach(VersionHandle vh, ctv->vh_range()) {
                ptTV.put("tv.details." + VersionHandle::indicatorToStr(vh.indicator), vh.substantive);
                tvToJSON(ctv->getVersionedTV(vh));
            }
*/
            break;
        }
    }

    // Incoming set
    HandleSeq incoming = as->getIncoming(h);
    ptree ptIncoming;
    for (uint i = 0; i < incoming.size(); i++) {
        ptree ptElement;
        ptElement.put("", std::to_string(incoming[i].value()));
        ptIncoming.push_back(std::make_pair("", ptElement));
    }

    // Outgoing set
    HandleSeq outgoing = as->getOutgoing(h);
    ptree ptOutgoing;
    for (uint i = 0; i < outgoing.size(); i++) {
        ptree ptElement;
        ptElement.put("", std::to_string(outgoing[i].value()));
        ptOutgoing.push_back(std::make_pair("", ptElement));
    }

    // Use Boost property trees for JSON serialization
    ptree ptAtoms;
    ptree pt;

    pt.put("handle", handleString);
    pt.put("type", typeNameString);
    pt.put("name", nameString);
    pt.put("attentionvalue.sti", stiString);
    pt.put("attentionvalue.lti", ltiString);
    pt.put("attentionvalue.vlti", vltiString);
    pt.add_child("truthvalue", ptTV);
    pt.add_child("outgoing", ptOutgoing);
    pt.add_child("incoming", ptIncoming);

    ptAtoms.add_child("atoms", pt);

    std::ostringstream buf;
    write_json (buf, ptAtoms, false); // false = don't use pretty print
    std::string json = buf.str();

    return json;
}

// TODO: Create tvToPtree method to convert a TruthValuePointer to a Boost Property Tree
// (Debug 'undefined symbol' issue with TruthValuePtr)
/*
ptree tvToPtree(TruthValuePtr tv)
{
    ptree pt;

    return pt;
}
*/

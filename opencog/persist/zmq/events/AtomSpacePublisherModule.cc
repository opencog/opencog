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

#include <opencog/server/Factory.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>
#include <opencog/util/platform.h>
#include <iostream>
#include <opencog/util/Config.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

// @todo: Remove zhelpers.hpp
#include "zhelpers.hpp"

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
    delete context;
    delete publisher;
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
    cout << payload;
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
    std::string vltiString = std::to_string(av->getVLTI());

    // TruthValue
    // @TODO: refer to this previous code:
    // https://github.com/opencog/opencog/pull/346/files#diff-01b35cec300185bd9a43ee545e50566bL123

    // @todo: move this into tvToJSON and fix pointer error encountered before
    TruthValuePtr tvp = as->getTV(h);
    //tvString = tvToJSON(tvp);
    std::string tvString = "";

    ptree ptTV;
    switch (tvp->getType())
    {
        case SIMPLE_TRUTH_VALUE:
            ptTV.put("type", "simple");
            ptTV.put("details.strength", tvp->getMean());
            ptTV.put("details.count", tvp->getCount());
            ptTV.put("details.confidence", tvp->getConfidence());

            break;

        case COUNT_TRUTH_VALUE:
            ptTV.put("type", "count");
            ptTV.put("details.strength", tvp->getMean());
            ptTV.put("details.count", tvp->getCount());
            ptTV.put("details.confidence", tvp->getConfidence());
            break;

        case INDEFINITE_TRUTH_VALUE:
            IndefiniteTruthValuePtr itv = IndefiniteTVCast(tvp);
            ptTV.put("type", "indefinite");
            ptTV.put("details.strength", itv->getMean());
            ptTV.put("details.L", itv->getL());
            ptTV.put("details.U", itv->getU());
            ptTV.put("details.confidence", itv->getConfidenceLevel());
            ptTV.put("details.diff", itv->getDiff());
            ptTV.put("details.symmetric", itv->isSymmetric());
            break;

// @todo: Resolve 'jump to case label' error
/*
        case COMPOSITE_TRUTH_VALUE:
            CompositeTruthValuePtr ctv = CompositeTVCast(tvp);
            ptTV.put("type", "composite");
            // @todo: Move tvToJSON and call it on tvp->getPrimaryTV()
            ptTV.put("details.primary", "");

            // @todo: Move tvToJSON and call it here

            //foreach(VersionHandle vh, ctv->vh_range()) {
            //    ptTV.put("tv.details." + VersionHandle::indicatorToStr(vh.indicator), vh.substantive);
            //    tvToJSON(ctv->getVersionedTV(vh));
            }
            break;
*/
    }

    std::ostringstream bufTV;
    write_json (bufTV, ptTV, false);
    tvString = bufTV.str();

    // https://github.com/opencog/opencog/pull/346/files#diff-01b35cec300185bd9a43ee545e50566bL130

    // Incoming
    //std::string incomingString = "";
    // @TODO:
    // https://github.com/opencog/opencog/pull/346/files#diff-01b35cec300185bd9a43ee545e50566bL113
    //HandleSeq incoming = as->getIncoming(h);
    //copy(incoming.begin(), incoming.end(), ostream_iterator<Handle>(cout, " "));
    // From previous REST API:


    HandleSeq incoming = as->getIncoming(h);
    ptree ptIncoming;
    cout << "\n\"incoming\":["; // @todo remove
    for (uint i = 0; i < incoming.size(); i++) {
        if (i != 0) cout << ",";     // @todo remove
        cout << incoming[i].value(); // @todo remove

        // @todo check how to insert these values
        // std::to_string(incoming[i].value())
    }
    // @todo remove
    cout << "],\n"; // << std::endl;


    // Outgoing
    //std::string outgoingString = "";
    // @TODO:
    // https://github.com/opencog/opencog/pull/346/files#diff-01b35cec300185bd9a43ee545e50566bL103

    // @todo why does this throw? the exception occurs at: line 443 of AtomTable.cc "Handle AtomTable::add(AtomPtr atom) throw (RuntimeException)"
    // And intermittently, it shows this error message:
    // terminate called after throwing an instance of 'concurrent_queue<std::shared_ptr<opencog::ASRequest> >::Canceled'
    // what():  std::exception

    /*
    HandleSeq outgoing = as->getOutgoing(h);
    ptree ptOutgoing;

    cout << "\n\"outgoing\":["; // @todo remove
    for (uint i = 0; i < outgoing.size(); i++) {
        if (i != 0) cout << ",";     // @todo remove
        cout << outgoing[i].value(); // @todo remove

        // @todo check how to insert these values
        // std::to_string(outgoing[i].value())
    }
    // @todo remove
    cout << "],\n"; // << std::endl;
    */

    // Use Boost property trees for JSON serialization
    ptree pt;
    pt.put("handle", handleString);
    pt.put("type", typeNameString);
    pt.put("name", nameString);
    pt.put("av.sti", stiString);
    pt.put("av.lti", ltiString);
    pt.put("av.vlti", vltiString);
    pt.put("tv", tvString);
    //pt.put_child("outgoing", ptOutgoing);//incomingString);
    pt.put_child("incoming", ptIncoming);//outgoingString);

    std::ostringstream buf;
    write_json (buf, pt, false);
    std::string json = buf.str();

    cout << json;

    return json;
}
/*
std::string tvToJSON(TruthValuePtr tvp)
{
    // @todo
}*/


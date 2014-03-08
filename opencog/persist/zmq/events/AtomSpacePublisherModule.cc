/*
 * opencog/persist/zmq/events/AtomSpacePublisherModule.cc
 *
 * Copyright (C) 2014 OpenCog Foundation
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
//#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/dynamics/attention/atom_types.h> // todo: remove?
#include "opencog/util/zhelpers.hpp"
#include <iostream>
#include <thread>
#include <iomanip>
#include <time.h>

#include <lib/json_spirit/json_spirit.h>

using namespace std;
using namespace json_spirit;
using namespace opencog;

DECLARE_MODULE(AtomSpacePublisherModule)

AtomSpacePublisherModule::AtomSpacePublisherModule(CogServer& cs) : Module(cs)
{
    logger().info("[AtomSpacePublisherModule] constructor");
    this->as = &cs.getAtomSpace();

    enableSignals();

    do_publisherEnableSignals_register();
    do_publisherDisableSignals_register();
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

    disableSignals();

    publisher->close();
    delete publisher;
    delete context;

    do_publisherEnableSignals_unregister();
    do_publisherDisableSignals_unregister();
}

void AtomSpacePublisherModule::enableSignals()
{
    if (!addAtomConnection.connected())
        addAtomConnection = as->addAtomSignal(
                boost::bind(&AtomSpacePublisherModule::atomAddSignal,
                            this, _1));
    if (!removeAtomConnection.connected())
    removeAtomConnection = as->removeAtomSignal(
                boost::bind(&AtomSpacePublisherModule::atomRemoveSignal,
                            this, _1));
    if (!TVChangedConnection.connected())
        TVChangedConnection = as->TVChangedSignal(
                boost::bind(&AtomSpacePublisherModule::TVChangedSignal,
                            this, _1, _2, _3));
    if (!AVChangedConnection.connected())
        AVChangedConnection = as->AVChangedSignal(
                boost::bind(&AtomSpacePublisherModule::AVChangedSignal,
                            this, _1, _2, _3));
    if (!AddAFConnection.connected())
        AddAFConnection = as->AddAFSignal(
                boost::bind(&AtomSpacePublisherModule::addAFSignal,
                            this, _1, _2, _3));
    if (!RemoveAFConnection.connected())
        RemoveAFConnection = as->RemoveAFSignal(
                boost::bind(&AtomSpacePublisherModule::removeAFSignal,
                            this, _1, _2, _3));
}

void AtomSpacePublisherModule::disableSignals()
{
    if (addAtomConnection.connected())
        addAtomConnection.disconnect();
    if (removeAtomConnection.connected())
        removeAtomConnection.disconnect();
    if (TVChangedConnection.connected())
        TVChangedConnection.disconnect();
    if (AVChangedConnection.connected())
        AVChangedConnection.disconnect();
    if (AddAFConnection.connected())
        AddAFConnection.disconnect();
    if (RemoveAFConnection.connected())
        RemoveAFConnection.disconnect();
}

void AtomSpacePublisherModule::InitZeroMQ()
{
    context = new zmq::context_t(1);

    std::thread proxyThread(&AtomSpacePublisherModule::proxy, this);
    proxyThread.detach();
}

void AtomSpacePublisherModule::proxy()
{
    zmq::socket_t * consumer;
    zmq::socket_t * pub;

    consumer = new zmq::socket_t(*context, ZMQ_PULL);
    pub = new zmq::socket_t(*context, ZMQ_PUB);

    std::string zmq_event_port = config().get("ZMQ_EVENT_PORT");
    bool zmq_use_public_ip = config().get_bool("ZMQ_EVENT_USE_PUBLIC_IP");
    std::string zmq_ip;
    if (zmq_use_public_ip)
        zmq_ip = "0.0.0.0";
    else
        zmq_ip = "*";

    try {
        consumer->bind("inproc://atomspace");
        pub->bind(("tcp://" + zmq_ip + ":" + zmq_event_port).c_str());
    } catch (zmq::error_t error) {
        std::cout << "ZeroMQ error: " << error.what() << std::endl;
        return;
    }

    // Forward messages from multiple producers on to the publisher
    while (1) {
        std::string address = s_recv(*consumer);
        std::string payload = s_recv(*consumer);
        s_sendmore(*pub, address);
        s_send(*pub, payload);
    }
}

void AtomSpacePublisherModule::sendMessage(std::string messageType,
                                           std::string payload)
{
    zmq::socket_t * socket = new zmq::socket_t(*context, ZMQ_PUSH);
    socket->connect("inproc://atomspace");
    s_sendmore (*socket, messageType);
    s_send (*socket, payload);
    socket->close();

    //cout << "[" << messageType << "] " << payload << endl;
}

void AtomSpacePublisherModule
::signalHandlerAdd(Handle h)
{
    sendMessage("add",
                atomMessage(atomToJSON(h)));
}

void AtomSpacePublisherModule
::signalHandlerRemove(AtomPtr atom)
{
    sendMessage("remove",
                atomMessage(atomToJSON(atom->getHandle())));
}

void AtomSpacePublisherModule
::signalHandlerAVChanged(const Handle& h,
                         const AttentionValuePtr& av_old,
                         const AttentionValuePtr& av_new)
{
        sendMessage("avChanged",
                    avMessage(atomToJSON(h),
                              avToJSON(av_old),
                              avToJSON(av_new)));
}

void AtomSpacePublisherModule
::signalHandlerTVChanged(const Handle& h,
                         const TruthValuePtr& tv_old,
                         const TruthValuePtr& tv_new)
{
    // Due to outstanding issue #394:
    //   https://github.com/opencog/opencog/issues/394
    // we check if the AttentionValue has actually changed
//    if (tv_old != tv_new) {
        sendMessage("tvChanged",
                    tvMessage(atomToJSON(h),
                              tvToJSON(tv_old),
                              tvToJSON(tv_new)));
//    }
}

void AtomSpacePublisherModule
::signalHandlerAddAF(const Handle& h,
                     const AttentionValuePtr& av_old,
                     const AttentionValuePtr& av_new)
{
    sendMessage("addAF",
                avMessage(atomToJSON(h),
                          avToJSON(av_old),
                          avToJSON(av_new)));
}

void AtomSpacePublisherModule
::signalHandlerRemoveAF(const Handle& h,
                        const AttentionValuePtr& av_old,
                        const AttentionValuePtr& av_new)
{
    sendMessage("removeAF",
                avMessage(atomToJSON(h),
                          avToJSON(av_old),
                          avToJSON(av_new)));
}

// Todo: Use a threadpool to process tasks off a work queue
// instead of creating a new thread for each signal handler

void AtomSpacePublisherModule::atomAddSignal(Handle h)
{
    std::thread handler(&AtomSpacePublisherModule::signalHandlerAdd,
                        this, h);
    handler.detach();
}

void AtomSpacePublisherModule::atomRemoveSignal(AtomPtr atom)
{
    // The AtomSpace API fires a dummy 'AVChangedSignal' signal when you remove
    // an atom:
    //   https://github.com/opencog/opencog/issues/394
    // After that bug is resolved, it may be necessary to check if the atom was
    // in the AttentionalFocus before being deleted, and then publish a
    // 'removeAF' signal

    // todo

    std::thread handler(&AtomSpacePublisherModule::signalHandlerRemove,
                        this, atom);
    handler.detach();
}

void AtomSpacePublisherModule::AVChangedSignal(const Handle& h,
                                               const AttentionValuePtr& av_old,
                                               const AttentionValuePtr& av_new)
{
    std::thread handler(&AtomSpacePublisherModule::signalHandlerAVChanged,
                            this, h, av_old, av_new);
    handler.detach();
}

void AtomSpacePublisherModule::TVChangedSignal(const Handle& h,
                                               const TruthValuePtr& tv_old,
                                               const TruthValuePtr& tv_new)
{
    // The AtomSpace API fires a dummy 'TVChangedSignal' signal when you add
    // a link:
    //   https://github.com/opencog/opencog/issues/394
    // Until that bug is fixed, this will ignore the simple case where the link
    // is created with no TruthValue defined. However, if the link is created
    // with a TruthValue defined, that dummy signal will still be published.

    // todo: investigate occasional wrong tv changed signals from the atomspace

    std::thread handler(&AtomSpacePublisherModule::signalHandlerTVChanged,
                        this, h, tv_old, tv_new);
    handler.detach();
}

void AtomSpacePublisherModule::addAFSignal(const Handle& h,
                                           const AttentionValuePtr& av_old,
                                           const AttentionValuePtr& av_new)
{
    std::thread handler(&AtomSpacePublisherModule::signalHandlerAddAF,
                            this, h, av_old, av_new);
    handler.detach();
}

void AtomSpacePublisherModule::removeAFSignal(const Handle& h,
                                              const AttentionValuePtr& av_old,
                                              const AttentionValuePtr& av_new)
{
    std::thread handler(&AtomSpacePublisherModule::signalHandlerRemoveAF,
                            this, h, av_old, av_new);
    handler.detach();
}

Object AtomSpacePublisherModule::atomToJSON(Handle h)
{
    // Type
    Type type = as->getType(h);
    std::string typeNameString = classserver().getTypeName(type);

    // Name
    std::string nameString = as->getName(h);

    // Handle
    std::string handle = std::to_string(h.value());

    // AttentionValue
    AttentionValuePtr av = as->getAV(h);
    Object jsonAV;
    jsonAV = avToJSON(av);

    // TruthValue
    TruthValuePtr tvp = as->getTV(h);
    Object jsonTV;
    jsonTV = tvToJSON(tvp);

    // Incoming set
    HandleSeq incomingHandles = as->getIncoming(h);
    Array incoming;
    for (uint i = 0; i < incomingHandles.size(); i++) {
        incoming.push_back(std::to_string(incomingHandles[i].value()));
    }

    // Outgoing set
    HandleSeq outgoingHandles = as->getOutgoing(h);
    Array outgoing;
    for (uint i = 0; i < outgoingHandles.size(); i++) {
        outgoing.push_back(std::to_string(outgoingHandles[i].value()));
    }

    Object jsonAtom;
    jsonAtom.push_back(Pair("handle", handle));
    jsonAtom.push_back(Pair("type", typeNameString));
    jsonAtom.push_back(Pair("name", nameString));
    jsonAtom.push_back(Pair("attentionvalue", jsonAV));
    jsonAtom.push_back(Pair("truthvalue", jsonTV));
    jsonAtom.push_back(Pair("outgoing", outgoing));
    jsonAtom.push_back(Pair("incoming", incoming));

    return jsonAtom;
}

Object AtomSpacePublisherModule::avToJSON(AttentionValuePtr av)
{
    Object json;

    json.push_back(Pair("sti", av->getSTI()));
    json.push_back(Pair("lti", av->getLTI()));
    json.push_back(Pair("vlti", av->getVLTI() != 0 ? true : false));

    return json;
}

Object AtomSpacePublisherModule::tvToJSON(TruthValuePtr tvp)
{
    Object json;
    Object jsonDetails;

    switch (tvp->getType()) {
        case SIMPLE_TRUTH_VALUE: {
            json.push_back(Pair("type", "simple"));
            jsonDetails.push_back(Pair("strength", tvp->getMean()));
//            jsonDetails.push_back(Pair("count", tvp->getCount()));
            jsonDetails.push_back(Pair("countString", std::to_string(tvp->getCount())));
            jsonDetails.push_back(Pair("confidence", tvp->getConfidence()));
            json.push_back(Pair("details", jsonDetails));
            break;
        }

        case COUNT_TRUTH_VALUE: {
            json.push_back(Pair("type", "count"));
            jsonDetails.push_back(Pair("strength", tvp->getMean()));
            jsonDetails.push_back(Pair("count", tvp->getCount()));
            jsonDetails.push_back(Pair("confidence", tvp->getConfidence()));
            json.push_back(Pair("details", jsonDetails));
            break;
        }

        case INDEFINITE_TRUTH_VALUE: {
            IndefiniteTruthValuePtr itv = IndefiniteTVCast(tvp);
            json.push_back(Pair("type", "indefinite"));
            jsonDetails.push_back(Pair("strength", itv->getMean()));
            jsonDetails.push_back(Pair("L", itv->getL()));
            jsonDetails.push_back(Pair("U", itv->getU()));
            jsonDetails.push_back(Pair("confidence", itv->getConfidenceLevel()));
            jsonDetails.push_back(Pair("diff", itv->getDiff()));
            jsonDetails.push_back(Pair("symmetric", itv->isSymmetric()));
            json.push_back(Pair("details", jsonDetails));
            break;
        }

        case NULL_TRUTH_VALUE:
        case NUMBER_OF_TRUTH_VALUE_TYPES: {
            break;
        }
    }

    return json;
}

std::string AtomSpacePublisherModule::atomMessage(Object jsonAtom)
{
    Object json;
    json.push_back(Pair("atom", jsonAtom));
    json.push_back(Pair("timestamp", time(0)));
    return jsonToString(json);
}

std::string AtomSpacePublisherModule::avMessage(
        Object jsonAtom, Object jsonAVOld, Object jsonAVNew)
{
    Object json;
    json.push_back(Pair("handle", find_value(jsonAtom, "handle")));
    json.push_back(Pair("avOld", jsonAVOld));
    json.push_back(Pair("avNew", jsonAVNew));
    json.push_back(Pair("atom", jsonAtom));
    json.push_back(Pair("timestamp", time(0)));
    return jsonToString(json);
}

std::string AtomSpacePublisherModule::tvMessage(
        Object jsonAtom, Object jsonTVOld, Object jsonTVNew)
{
    Object json;
    json.push_back(Pair("handle", find_value(jsonAtom, "handle")));
    json.push_back(Pair("tvOld", jsonTVOld));
    json.push_back(Pair("tvNew", jsonTVNew));
    json.push_back(Pair("atom", jsonAtom));
    json.push_back(Pair("timestamp", time(0)));
    return jsonToString(json);
}

std::string AtomSpacePublisherModule::jsonToString(Object json)
{
    return write_formatted(json);
}

std::string AtomSpacePublisherModule
::do_publisherEnableSignals(Request *dummy, std::list<std::string> args)
{
    enableSignals();

    return "AtomSpace Publisher signals have been enabled.\n";
}

std::string AtomSpacePublisherModule
::do_publisherDisableSignals(Request *dummy, std::list<std::string> args)
{
    disableSignals();

    return "AtomSpace Publisher signals have been disabled.\n";
}

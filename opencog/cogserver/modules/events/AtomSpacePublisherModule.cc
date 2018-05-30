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

#include <iomanip>
#include <iostream>
#include <thread>
#include <time.h>

#include <lib/json_spirit/json_spirit.h>
#include <lib/zmq/zhelpers.hpp>
#include <tbb/task.h>
#include <tbb/concurrent_queue.h>

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/tbb.h>

#include <opencog/atoms/proto/NameServer.h>

#include <opencog/truthvalue/ProbabilisticTruthValue.h>
#include <opencog/truthvalue/FuzzyTruthValue.h>
#include <opencog/truthvalue/IndefiniteTruthValue.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/cogserver/server/CogServer.h>
#include "AtomSpacePublisherModule.h"

using namespace std;
using namespace std::placeholders;
using namespace json_spirit;
using namespace opencog;

DECLARE_MODULE(AtomSpacePublisherModule)

AtomSpacePublisherModule::AtomSpacePublisherModule(CogServer& cs) : Module(cs)
{
    logger().info("[AtomSpacePublisherModule] constructor");
    this->as = &cs.getAtomSpace();
    _attention_bank = &attentionbank(as);

    _remove_atom_connection = 0;
    _add_atom_connection = 0;
    _tvchange_connection = 0;
    _avchange_connection = 0;
    _add_af_connection = 0;
    _remove_af_connection = 0;
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

    // Shut down the ZeroMQ proxy loop
    message_t message;
    message.type = "CONTROL";
    message.payload = "TERMINATE";
    queue.push(message);
    context->close();

    do_publisherEnableSignals_unregister();
    do_publisherDisableSignals_unregister();
}

void AtomSpacePublisherModule::enableSignals()
{
    if (0 == _add_atom_connection)
    {
        _add_atom_signal = &as->atomAddedSignal();
        _add_atom_connection = _add_atom_signal->connect(
            std::bind(
            &AtomSpacePublisherModule::atomAddSignal, this, _1));
    }
    if (0 == _remove_atom_connection)
    {
        _remove_atom_signal = &as->atomRemovedSignal();
        _remove_atom_connection = _remove_atom_signal->connect(
            std::bind(
            &AtomSpacePublisherModule::atomRemoveSignal, this, _1));
    }
    if (0 == _tvchange_connection)
    {
        _tvchange_signal = &as->TVChangedSignal();
        _tvchange_connection = _tvchange_signal->connect(
            std::bind(
            &AtomSpacePublisherModule::TVChangedSignal, this, _1, _2, _3));
    }
    if (0 == _avchange_connection)
    {
        _avchange_signal = &_attention_bank->getAVChangedSignal();
        _avchange_connection = _avchange_signal->connect(
            std::bind(&AtomSpacePublisherModule::AVChangedSignal,
                        this, _1, _2, _3));
    }
    if (0 == _add_af_connection)
    {
        _add_af_signal = &_attention_bank->AddAFSignal();
        _add_af_connection = _add_af_signal->connect(
            std::bind(&AtomSpacePublisherModule::addAFSignal,
                        this, _1, _2, _3));
    }
    if (0 == _remove_af_connection)
    {
        _remove_af_signal = &_attention_bank->RemoveAFSignal();
        _remove_af_connection = _remove_af_signal->connect(
            std::bind(&AtomSpacePublisherModule::removeAFSignal,
                        this, _1, _2, _3));
    }
}

void AtomSpacePublisherModule::disableSignals()
{
    if (_add_atom_connection)
    {
        _add_atom_signal->disconnect(_add_atom_connection);
        _add_atom_connection = 0;
    }
    if (_remove_atom_connection)
    {
        _remove_atom_signal->disconnect(_remove_atom_connection);
        _remove_atom_connection = 0;
    }
    if (_tvchange_connection)
    {
        _tvchange_signal->disconnect(_tvchange_connection);
        _tvchange_connection = 0;
    }
    if (_avchange_connection)
    {
        _avchange_signal->disconnect(_avchange_connection);
        _avchange_connection = 0;
    }
    if (_add_af_connection)
    {
        _add_af_signal->disconnect(_add_af_connection);
        _add_af_connection = 0;
    }
    if (_remove_af_connection)
    {
        _remove_af_signal->disconnect(_remove_af_connection);
        _remove_af_connection = 0;
    }
}

void AtomSpacePublisherModule::InitZeroMQ()
{
    context = new zmq::context_t(1);
    std::thread proxyThread(&AtomSpacePublisherModule::proxy, this);
    proxyThread.detach();
}

void AtomSpacePublisherModule::proxy()
{
    zmq::socket_t pub(*context, ZMQ_PUB);
    pub.setsockopt(ZMQ_SNDHWM, &HWM, sizeof(HWM));

    std::string zmq_event_port = config().get("ZMQ_EVENT_PORT");
    bool zmq_use_public_ip = config().get_bool("ZMQ_EVENT_USE_PUBLIC_IP");
    std::string zmq_ip;
    if (zmq_use_public_ip)
    {
        zmq_ip = "0.0.0.0";
    }
    else
    {
        zmq_ip = "*";
    }

    try
    {
        pub.bind(("tcp://" + zmq_ip + ":" + zmq_event_port).c_str());
    }
    catch (zmq::error_t error)
    {
        std::cout << "ZeroMQ error: " << error.what() << std::endl;
        return;
    }

    // Concurrent queue uses blocking pop operation to demultiplex messages
    // received from multithreaded TBB worker tasks and forward them to the
    // ZeroMQ publisher socket
    bool active = true;
    while (active)
    {
        message_t message;
        queue.pop(message);

        if (message.type == "CONTROL")
        {
            if (message.payload == "TERMINATE")
            {
                active = false;
            }
        }
        else
        {
            s_sendmore(pub, message.type);
            s_send(pub, message.payload);
        }
    }
}

void AtomSpacePublisherModule::sendMessage(std::string messageType,
                                           std::string payload)
{
    message_t message;
    message.type = messageType;
    message.payload = payload;
    queue.push(message);
}

void AtomSpacePublisherModule::atomAddSignal(Handle h)
{
    tbb_enqueue_lambda([=] {
       sendMessage("add", atomMessage(atomToJSON(h)));
    });
}

void AtomSpacePublisherModule::atomRemoveSignal(AtomPtr atom)
{
    tbb_enqueue_lambda([=] {
        sendMessage("remove", atomMessage(atomToJSON(atom->get_handle())));
    });
}

void AtomSpacePublisherModule::AVChangedSignal(const Handle& h,
                                               const AttentionValuePtr& av_old,
                                               const AttentionValuePtr& av_new)
{
    tbb_enqueue_lambda([=] {
        sendMessage("avChanged", avMessage(atomToJSON(h),
                                           avToJSON(av_old),
                                           avToJSON(av_new)));
    });
}

void AtomSpacePublisherModule::TVChangedSignal(const Handle& h,
                                               const TruthValuePtr& tv_old,
                                               const TruthValuePtr& tv_new)
{
    tbb_enqueue_lambda([=] {
        sendMessage("tvChanged", tvMessage(atomToJSON(h),
                                           tvToJSON(tv_old),
                                           tvToJSON(tv_new)));
    });
}

void AtomSpacePublisherModule::addAFSignal(const Handle& h,
                                           const AttentionValuePtr& av_old,
                                           const AttentionValuePtr& av_new)
{
    tbb_enqueue_lambda([=] {
        sendMessage("addAF", avMessage(atomToJSON(h),
                                       avToJSON(av_old),
                                       avToJSON(av_new)));
    });
}

void AtomSpacePublisherModule::removeAFSignal(const Handle& h,
                                              const AttentionValuePtr& av_old,
                                              const AttentionValuePtr& av_new)
{
    tbb_enqueue_lambda([=] {
        sendMessage("removeAF", avMessage(atomToJSON(h),
                                          avToJSON(av_old),
                                          avToJSON(av_new)));
    });
}


Object AtomSpacePublisherModule::atomToJSON(Handle h)
{
    // Type
    Type type = h->get_type();
    std::string typeNameString = nameserver().getTypeName(type);

    // Name
    std::string nameString = h->get_name();

    // Handle
    std::string handle = std::to_string(h.value());

    // AttentionValue
    AttentionValuePtr av = get_av(h);
    Object jsonAV;
    jsonAV = avToJSON(av);

    // TruthValue
    TruthValuePtr tvp = h->getTruthValue();
    Object jsonTV;
    jsonTV = tvToJSON(tvp);

    // Incoming set
    HandleSeq incomingHandles;
    h->getIncomingSet(back_inserter(incomingHandles));
    Array incoming;
    for (uint i = 0; i < incomingHandles.size(); i++) {
        incoming.push_back(std::to_string(incomingHandles[i].value()));
    }

    // Outgoing set
    Array outgoing;
    if (h->is_link()) {
        HandleSeq outgoingHandles = h->getOutgoingSet();
        for (uint i = 0; i < outgoingHandles.size(); i++) {
            outgoing.push_back(std::to_string(outgoingHandles[i].value()));
        }
    }

    Object json;
    json.push_back(Pair("handle", handle));
    json.push_back(Pair("type", typeNameString));
    json.push_back(Pair("name", nameString));
    json.push_back(Pair("attentionvalue", jsonAV));
    json.push_back(Pair("truthvalue", jsonTV));
    json.push_back(Pair("outgoing", outgoing));
    json.push_back(Pair("incoming", incoming));

    return json;
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
    Type tvt = tvp->get_type();


    if (tvt == SIMPLE_TRUTH_VALUE) {
        json.push_back(Pair("type", "simple"));
        jsonDetails.push_back(Pair("strength", tvp->get_mean()));
        jsonDetails.push_back(Pair("count", tvp->get_count()));
        jsonDetails.push_back(Pair("confidence", tvp->get_confidence()));
        json.push_back(Pair("details", jsonDetails));
    }
    else if (tvt == COUNT_TRUTH_VALUE) {
        json.push_back(Pair("type", "count"));
        jsonDetails.push_back(Pair("strength", tvp->get_mean()));
        jsonDetails.push_back(Pair("count", tvp->get_count()));
        jsonDetails.push_back(Pair("confidence", tvp->get_confidence()));
        json.push_back(Pair("details", jsonDetails));
    }
    else if (tvt == INDEFINITE_TRUTH_VALUE) {
        IndefiniteTruthValuePtr itv = IndefiniteTVCast(tvp);
        json.push_back(Pair("type", "indefinite"));
        jsonDetails.push_back(Pair("strength", itv->get_mean()));
        jsonDetails.push_back(Pair("L", itv->getL()));
        jsonDetails.push_back(Pair("U", itv->getU()));
        jsonDetails.push_back(Pair("confidence", itv->getConfidenceLevel()));
        jsonDetails.push_back(Pair("diff", itv->getDiff()));
        jsonDetails.push_back(Pair("symmetric", itv->isSymmetric()));
        json.push_back(Pair("details", jsonDetails));
    }
    else if (tvt == PROBABILISTIC_TRUTH_VALUE) {
        json.push_back(Pair("type", "probabilistic"));
        jsonDetails.push_back(Pair("strength", tvp->get_mean()));
        jsonDetails.push_back(Pair("count", tvp->get_count()));
        jsonDetails.push_back(Pair("confidence", tvp->get_confidence()));
        json.push_back(Pair("details", jsonDetails));
    }
    else if (tvt == FUZZY_TRUTH_VALUE) {
        json.push_back(Pair("type", "fuzzy"));
        jsonDetails.push_back(Pair("strength", tvp->get_mean()));
        jsonDetails.push_back(Pair("count", tvp->get_count()));
        jsonDetails.push_back(Pair("confidence", tvp->get_confidence()));
        json.push_back(Pair("details", jsonDetails));
    }
    else {
        throw InvalidParamException(TRACE_INFO,
            "Invalid TruthValue Type parameter.");
    }

    return json;
}

std::string AtomSpacePublisherModule::atomMessage(Object jsonAtom)
{
    Object json;
    json.push_back(Pair("atom", jsonAtom));
    json.push_back(Pair("timestamp", (boost::uint64_t)time(0)));
    return write_formatted(json);
}

std::string AtomSpacePublisherModule::avMessage(
        Object jsonAtom, Object jsonAVOld, Object jsonAVNew)
{
    Object json;
    json.push_back(Pair("handle", find_value(jsonAtom, "handle")));
    json.push_back(Pair("avOld", jsonAVOld));
    json.push_back(Pair("avNew", jsonAVNew));
    json.push_back(Pair("atom", jsonAtom));
    json.push_back(Pair("timestamp", (boost::uint64_t)time(0)));
    return write_formatted(json);
}

std::string AtomSpacePublisherModule::tvMessage(
        Object jsonAtom, Object jsonTVOld, Object jsonTVNew)
{
    Object json;
    json.push_back(Pair("handle", find_value(jsonAtom, "handle")));
    json.push_back(Pair("tvOld", jsonTVOld));
    json.push_back(Pair("tvNew", jsonTVNew));
    json.push_back(Pair("atom", jsonAtom));
    json.push_back(Pair("timestamp", (boost::uint64_t)time(0)));
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

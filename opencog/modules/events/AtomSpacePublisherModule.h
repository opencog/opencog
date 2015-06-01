/*
 * opencog/persist/zmq/events/AtomSpacePublisherModule.h
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

#ifndef _OPENCOG_ATOMSPACE_PUBLISHER_MODULE_H
#define _OPENCOG_ATOMSPACE_PUBLISHER_MODULE_H

#include <string>
#include <opencog/server/Agent.h>
#include <opencog/server/Module.h>
#include <opencog/server/CogServer.h>
#include <lib/json_spirit/json_spirit.h>
#include <tbb/task.h>
#include <tbb/concurrent_queue.h>
#include <opencog/util/tbb.h>
#include <lib/zmq/zhelpers.hpp>

using namespace json_spirit;

namespace opencog
{

class CogServer;

/**
 * The AtomSpacePublisherModule class publishes AtomSpace change events across
 * the network using ZeroMQ to allow for external clients to receive updates
 * from the AtomSpace via a publish/subscribe pattern.
 *
 * API documentation is in: README.md
 *
 * Clients can subscribe to the events by subscribing to the ZeroMQ socket
 * defined in the ZMQ_EVENT_PORT parameter set in the OpenCog configuration
 * file.
 *
 * Supported events are:
 *
 *   add        (Atom added)
 *   remove     (Atom removed)
 *   tvChanged  (Atom TruthValue changed)
 *   avChanged  (Atom AttentionValue changed)
 *   addAF      (Atom AttentionValue changed and entered the AttentionalFocus)
 *   removeAF   (Atom AttentionValue changed and exited the AttentionalFocus)
 *
 * Architecture:
 *   - Uses Intel TBB (Threaded Building Blocks), Boost Signals2 and ZeroMQ
 *   - The Boost Signals2 slots receive atomspace events and can be multithreaded
 *   - Events are enqueued as a pending TBB task
 *   - TBB scheduler schedules the tasks across available processor cores
 *   - Tasks serialize the atomspace event into a standard JSON message format
 *   - Serialized output is forwarded to a TBB concurrent queue
 *   - Proxy accesses the concurrent queue using a blocking pop operation to 
 *     demultiplex messages and forward them to the ZeroMQ publisher socket
 **/
class AtomSpacePublisherModule;
typedef std::shared_ptr<AtomSpacePublisherModule> AtomSpacePublisherModulePtr;

struct message_t {
    std::string type;
    std::string payload;
};

// High water mark for publisher socket
const int HWM = 10000000;

class AtomSpacePublisherModule : public Module
{
    private:
        AtomSpace* as;
        boost::signals2::connection removeAtomConnection;
        boost::signals2::connection addAtomConnection;
        boost::signals2::connection TVChangedConnection;
        boost::signals2::connection AVChangedConnection;
        boost::signals2::connection AddAFConnection;
        boost::signals2::connection RemoveAFConnection;
        void enableSignals();
        void disableSignals();

        // TBB
        tbb::concurrent_bounded_queue<message_t> queue;
        
        // ZeroMQ
        zmq::context_t * context;
        void InitZeroMQ();
        void proxy();

        void sendMessage(std::string messageType, std::string payload);
        std::string atomMessage(Object jsonAtom);
        std::string avMessage(Object jsonAtom, Object jsonAVOld, Object jsonAVNew);
        std::string tvMessage(Object jsonAtom, Object jsonTVOld, Object jsonTVNew);
        Object atomToJSON(Handle h);
        Object tvToJSON(TruthValuePtr tv);
        Object avToJSON(AttentionValuePtr av);

        DECLARE_CMD_REQUEST(AtomSpacePublisherModule, "publisher-enable-signals",
           do_publisherEnableSignals,
           "Enable AtomSpace event publishing",
           "Usage: publisher-enable-signals",
           false, false)

        DECLARE_CMD_REQUEST(AtomSpacePublisherModule, "publisher-disable-signals",
           do_publisherDisableSignals,
           "Disable AtomSpace event publishing",
           "Usage: publisher-disable-signals",
           false, false)

    public:
        AtomSpacePublisherModule(CogServer&);
        virtual ~AtomSpacePublisherModule();
        virtual void run();

        static const char *id(void);
        virtual void init(void);

        void atomAddSignal(Handle h);
        void atomRemoveSignal(AtomPtr atom);
        void AVChangedSignal(const Handle& h,
                             const AttentionValuePtr& av_old,
                             const AttentionValuePtr& av_new);
        void TVChangedSignal(const Handle& h,
                             const TruthValuePtr& tv_old,
                             const TruthValuePtr& tv_new);
        void addAFSignal(const Handle& h,
                         const AttentionValuePtr& av_old,
                         const AttentionValuePtr& av_new);
        void removeAFSignal(const Handle& h,
                         const AttentionValuePtr& av_old,
                         const AttentionValuePtr& av_new);
};

}

#endif

/*
 * opencog/persist/zmq/events/AtomSpacePublisherModule.h
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

#ifndef _OPENCOG_ATOMSPACE_PUBLISHER_MODULE_H
#define _OPENCOG_ATOMSPACE_PUBLISHER_MODULE_H

#include <string>

#include <opencog/server/Agent.h>
#include <opencog/server/Module.h>
#include <opencog/server/CogServer.h>

#include <boost/property_tree/ptree.hpp>
using boost::property_tree::ptree;

#include "opencog/util/zhelpers.hpp"

namespace opencog
{

class CogServer;

/**
 * The AtomSpacePublisherModule class publishes AtomSpace change events across the network using ZeroMQ
 * to allow for external clients to receive updates from the AtomSpace via a publish/subscribe pattern.
 *
 * Full documentation is available in README.md
 *
 * Clients can subscribe to the events by subscribing to the ZeroMQ socket defined in the
 * ZMQ_EVENT_PORT parameter set in the OpenCog configuration file.
 *
 * Supported events are:
 *
 *   add
 *   remove
 *   tvchanged
 *   avchanged
 *
 * The message is a JSON-formatted string with the following structure:
 *   http://wiki.opencog.org/w/AtomSpace_Event_Publisher#Message_format
 *
 **/
class AtomSpacePublisherModule;
typedef std::shared_ptr<AtomSpacePublisherModule> AtomSpacePublisherModulePtr;

class AtomSpacePublisherModule : public Module
{
    private:
        AtomSpace* as;
        boost::signals2::connection removeAtomConnection;
        boost::signals2::connection addAtomConnection;
        boost::signals2::connection TVChangedConnection;
        boost::signals2::connection AVChangedConnection;

        zmq::context_t * context;
        zmq::socket_t * publisher;

        void InitZeroMQ();

        ptree atomToPtree(Handle h);
        ptree tvToPtree(TruthValuePtr tv);
        ptree avToPtree(AttentionValuePtr av);
        std::string atomMessage(ptree ptAtom);
        std::string avMessage(ptree ptAtom, ptree ptAVOld, ptree ptAVNew);
        std::string tvMessage(ptree ptAtom, ptree ptTVOld, ptree ptTVNew);
        std::string ptToJSON(ptree pt);

    public:
        AtomSpacePublisherModule(CogServer&);
        virtual ~AtomSpacePublisherModule();
        virtual void run();

        static const char *id(void);
        virtual void init(void);

        void atomAddSignal(Handle h);
        void atomRemoveSignal(AtomPtr atom);
        void AVChangedSignal(const Handle& h, const AttentionValuePtr& av_old, const AttentionValuePtr& av_new);
        void TVChangedSignal(const Handle& h, const TruthValuePtr& tv_old, const TruthValuePtr& tv_new);
};

}

#endif

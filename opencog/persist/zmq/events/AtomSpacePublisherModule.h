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
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>

#include <opencog/server/Request.h>
#include <opencog/server/CogServer.h>

#include "zhelpers.hpp"

namespace opencog
{

class CogServer;

/**
 * The AtomSpacePublisherModule class publishes AtomSpace change events across the network using
 * ZeroMQ to allow for external clients to interface with the AtomSpace via a publish/subscribe pattern.
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
 *   http://wiki.opencog.org/w/REST_API#Format_of_Atom_object
 **/
class AtomSpacePublisherModule;
typedef std::shared_ptr<AtomSpacePublisherModule> AtomSpacePublisherModulePtr;

class AtomSpacePublisherModule : public Module
{
    private:
        AtomSpace* as;
        boost::signals::connection removeAtomConnection;
        boost::signals::connection addAtomConnection;
        boost::signals::connection TVChangedConnection;
        boost::signals::connection AVChangedConnection;

        zmq::context_t * context;
        zmq::socket_t * publisher;

        void InitZeroMQ();

        std::string atomToJSON(Handle h);
        std::string tvToJSON(TruthValuePtr& tvp);

    public:
        AtomSpacePublisherModule(CogServer&);
        virtual ~AtomSpacePublisherModule();
        virtual void run();

        static const char *id(void);
        virtual void init(void);

        void handleAddSignal(Handle h);
        void atomRemoveSignal(AtomPtr atom);
        void AVChangedSignal(const Handle& h, const AttentionValuePtr& av_old, const AttentionValuePtr& av_new);
        void TVChangedSignal(const Handle& h, const TruthValuePtr& tv_old, const TruthValuePtr& tv_new);
}; // class

} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_PUBLISHER_MODULE_H



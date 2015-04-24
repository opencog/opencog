/*
 * PLNModule.cc
 *
 * Copyright (C) 2015 by OpenCog Foundation
 * Written by Misgana Bayetta <misgana.bayetta@gmail.com>
 * All Rights Reserved
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
#include <opencog/server/CogServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <tbb/task.h>
#include <opencog/util/tbb.h>
#include <opencog/util/Config.h>
#include "PLNModule.h"
#include "DefaultForwardChainerCB.h"
#include "ForwardChainer.h"

using namespace opencog;

DECLARE_MODULE(PLNModule);

PLNModule::PLNModule(CogServer& cs) :
        Module(cs)
{
    as_ = &cs.getAtomSpace();

    enum StartingCondition {
        ATOM_ADDED = 1, ADDED_TO_AF, BOTH
    };
    StartingCondition cond = static_cast<StartingCondition>(config().get_int(
            "RULE_ENGINE_TRIGGERED_ON"));

    switch (cond) {
    case ATOM_ADDED:
        add_atom_connection_ = as_->addAtomSignal(
                boost::bind(&PLNModule::add_atom_signal, this, _1));
        logger().info("Listening on ATOM_ADDED event.");
        break;
    case ADDED_TO_AF:
        add_af_connection_ = as_->AddAFSignal(
                boost::bind(&PLNModule::add_af_signal, this, _1, _2, _3));
        logger().info("Listening on ADDED_TO_AF event.");
        break;
    case BOTH:
        add_atom_connection_ = as_->addAtomSignal(
                boost::bind(&PLNModule::add_atom_signal, this, _1));
        add_af_connection_ = as_->AddAFSignal(
                boost::bind(&PLNModule::add_af_signal, this, _1, _2, _3));
        logger().info("Listening on BOTH(ATOM_ADDED and ADDED_TO_AF) event.");
        break;
    default:
        break;
    }
}

PLNModule::~PLNModule()
{
    logger().info("Destroying PLNModule instance.");
    add_atom_connection_.disconnect();
    add_af_connection_.disconnect();
}

void PLNModule::run()
{

}

void PLNModule::init()
{
    logger().info("Initializing PLNModule.");
}
/*
 * Create a task to handle the add_af_signal_handler event to avoid blocking.
 */
void PLNModule::add_af_signal(const Handle& source,
                              const AttentionValuePtr& av_old,
                              const AttentionValuePtr& av_new)
{
    tbb_enqueue_lambda([=] {
        add_af_signal_handler(source, av_old, av_new);
    });
}

/**
 * Whenever an atom enters in to the attentional focus,  start PLN reasoning.
 * This can be seen as a reactive(event-condition-action) type of rule engine
 * scenario.
 */
void PLNModule::add_af_signal_handler(const Handle& h,
                                      const AttentionValuePtr& av_old,
                                      const AttentionValuePtr& av_new)
{
    //!start the chainer xxx more code here
    DefaultForwardChainerCB dfcb(as_);
    ForwardChainer fc(as_);
    fc.do_chain(dfcb, h);
}
/*
 * Create a task to handle the add_atom_signal_handler event to avoid blocking.
 */
void PLNModule::add_atom_signal(const Handle& new_atom)
{
    tbb_enqueue_lambda([=] {
        add_atom_signal_handler(new_atom);
    });
}

/**
 * Whenever a new atom is added, start PLN reasoning.
 */
void PLNModule::add_atom_signal_handler(const Handle& h)
{
    //!Start the chainer xxx more code here/
    DefaultForwardChainerCB dfcb(as_);
    ForwardChainer fc(as_);
    fc.do_chain(dfcb, h);
}

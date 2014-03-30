/*
* opencog/dynamics/attention/HebbianCreationModule.h
*
* Copyright (C) 2014 Cosmo Harrigan
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

#ifndef _OPENCOG_HEBBIAN_CREATION_MODULE_H
#define _OPENCOG_HEBBIAN_CREATION_MODULE_H

#include <algorithm>

#include <opencog/server/Module.h>
#include <opencog/server/CogServer.h>
#include <tbb/task.h>
#include <opencog/util/tbb.h>

namespace opencog
{

class HebbianCreationModule;
typedef std::shared_ptr<HebbianCreationModule> HebbianCreationModulePtr;

/**
* The HebbianCreationModule creates AsymmetricHebbianLinks between
* atoms in the AttentionalFocus for updating by the HebbianUpdatingAgent.
*
* It is notified of atoms that are added to the AttentionalFocus
* by registering a slot with the Boost Signals2 events exposed by
* the AtomSpace. When this slot is called, a handler task is enqueued
* to do the work while allowing execution to continue.
**/
class HebbianCreationModule : public Module
{
   private:
       AtomSpace* as;
       boost::signals2::connection addAFConnection;

   public:
       HebbianCreationModule(CogServer&);
       virtual ~HebbianCreationModule();
       virtual void run();

       static const char *id(void);
       virtual void init(void);

       void addAFSignal(const Handle& h, const AttentionValuePtr& av_old,
                        const AttentionValuePtr& av_new);
       void addAFSignalHandler(const Handle& h, const AttentionValuePtr& av_old,
                               const AttentionValuePtr& av_new);
};

}

#endif

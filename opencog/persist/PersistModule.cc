/*
 * opencog/persist/PersistModule.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include "PersistModule.h"

#include <opencog/server/CogServer.h>

using namespace opencog;

DECLARE_MODULE(PersistModule);

PersistModule::PersistModule(void) : store(NULL)
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerRequest(SQLOpenRequest::info().id,  &sqlopenFactory);
    cogserver.registerRequest(sqlcloseRequest::info().id, &sqlcloseFactory);
    cogserver.registerRequest(SQLLoadRequest::info().id,  &sqlloadFactory);
    cogserver.registerRequest(SQLStoreRequest::info().id, &sqlstoreFactory);
}

PersistModule::~PersistModule()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.unregisterRequest(SQLOpenRequest::info().id);
    cogserver.unregisterRequest(sqlcloseRequest::info().id);
    cogserver.unregisterRequest(SQLLoadRequest::info().id);
    cogserver.unregisterRequest(SQLStoreRequest::info().id);
}

void PersistModule::init(void)
{
}

void PersistModule::setStore(AtomStorage* as)
{
    store = as;
}

AtomStorage* PersistModule::getStore(void)
{
    return store;
}

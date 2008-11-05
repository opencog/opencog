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
#include "AtomStorage.h"

#include <opencog/server/CogServer.h>
#include <opencog/atomspace/AtomSpace.h>

using namespace opencog;

DECLARE_MODULE(PersistModule);

PersistModule::PersistModule(void) : store(NULL)
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerRequest(sqlopenRequest::info().id,  &sqlopenFactory);
    cogserver.registerRequest(sqlcloseRequest::info().id, &sqlcloseFactory);
    cogserver.registerRequest(sqlloadRequest::info().id,  &sqlloadFactory);
    cogserver.registerRequest(sqlstoreRequest::info().id, &sqlstoreFactory);
}

PersistModule::~PersistModule()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.unregisterRequest(sqlopenRequest::info().id);
    cogserver.unregisterRequest(sqlcloseRequest::info().id);
    cogserver.unregisterRequest(sqlloadRequest::info().id);
    cogserver.unregisterRequest(sqlstoreRequest::info().id);
}

void PersistModule::init(void)
{
}

std::string PersistModule::do_close(std::list<std::string> args)
{
	if (!args.empty()) 
		return "sqlclose: Wrong num args";

	if (store == NULL)
		return "sqlclose: database not open";

	delete store;
	store = NULL;
	return "database closed";
}

std::string PersistModule::do_load(std::list<std::string> args)
{
	if (!args.empty()) 
		return "sqlload: Wrong num args";

	if (store == NULL)
		return "sqlload: database not open";

	store->load(atomtable());

	return "database load started";
}


std::string PersistModule::do_open(std::list<std::string> args)
{
	if (args.size() != 3)
		return "sqlload: Wrong num args";

	std::string dbname   = args.front(); args.pop_front();
	std::string username = args.front(); args.pop_front();
	std::string auth	   = args.front(); args.pop_front();

	store = new AtomStorage(dbname, username, auth);
	if (!store)
		return "sqlopen: Unable to open the database";

	return "database opened";
}

std::string PersistModule::do_store(std::list<std::string> args)
{
	if (!args.empty()) 
		return "sqlstore: Wrong num args";

	if (store == NULL)
		return "sqlstore: database not open";

	store->store(atomtable());

	return "database store started";
}

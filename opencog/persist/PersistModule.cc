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

void PersistModule::setStore(AtomStorage* as)
{
    store = as;
}

AtomStorage* PersistModule::getStore(void)
{
    return store;
}


static std::string on_close(PersistModule* persist, std::list<std::string> args)
{
	if (!args.empty()) 
		return "sqlclose: Wrong num args";

	AtomStorage* store = persist->getStore();
	if (store == NULL)
		return "sqlclose: database not open";

	delete store;
	persist->setStore(NULL);
	return "database closed";
}

bool sqlcloseRequest::execute()
{
    logger().debug("[sqlcloseRequest] execute");
    std::ostringstream oss;

    CogServer& cogserver = static_cast<CogServer&>(server());
    PersistModule* persist =
        static_cast<PersistModule*>(cogserver.getModule("opencog::PersistModule"));
    std::string rs = on_close(persist, _parameters);
    oss << rs << std::endl;

    if (_mimeType == "text/plain")
        send(oss.str());

    return true;
}

bool sqlloadRequest::execute()
{
    logger().debug("[sqlloadRequest] execute");
    std::ostringstream oss;

    if (_parameters.empty()) {
        CogServer& cogserver = static_cast<CogServer&>(server());
        PersistModule* persist =
            static_cast<PersistModule*>(cogserver.getModule("opencog::PersistModule"));
        AtomStorage* store = persist->getStore();
        if (store == NULL) oss << "error: invalid SQL storage" << std::endl;
        else store->load(const_cast<AtomTable&>(cogserver.getAtomSpace()->getAtomTable()));
    } else oss << info().help << std::endl;

    if (_mimeType == "text/plain")
        send(oss.str());

    return true;
}


bool sqlopenRequest::execute()
{
    logger().debug("[sqlopenRequest] execute");
    std::ostringstream oss;

    if (_parameters.size() == 3) {
        std::string dbname   = _parameters.front(); _parameters.pop_front();
        std::string username = _parameters.front(); _parameters.pop_front();
        std::string auth     = _parameters.front(); _parameters.pop_front();

        AtomStorage* store = new AtomStorage(dbname, username, auth);
        if (store) {
            CogServer& cogserver = static_cast<CogServer&>(server());
            PersistModule* persist =
                static_cast<PersistModule*>(cogserver.getModule("opencog::PersistModule"));
            persist->setStore(store);

            oss << "done" << std::endl;
        } else oss << "error: unable to open db \"" << dbname << "\"" << std::endl;
    } else oss << info().help << std::endl;

    if (_mimeType == "text/plain")
        send(oss.str());

    return true;
}

bool sqlstoreRequest::execute()
{
    logger().debug("[sqlstoreRequest] execute");
    std::ostringstream oss;

    if (_parameters.empty()) {
        CogServer& cogserver = static_cast<CogServer&>(server());
        PersistModule* persist =
            static_cast<PersistModule*>(cogserver.getModule("opencog::PersistModule"));
        AtomStorage* store = persist->getStore();
        if (store == NULL) oss << "error: invalid SQL storage" << std::endl;
        else store->store(cogserver.getAtomSpace()->getAtomTable());
    } else oss << info().help << std::endl;

    if (_mimeType == "text/plain")
        send(oss.str());

    return true;
}

/*
 * opencog/persist/SQLLoadRequest.cc
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

#include "SQLLoadRequest.h"

#include <sstream>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/persist/AtomStorage.h>
#include <opencog/persist/PersistModule.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

using namespace opencog;

SQLLoadRequest::SQLLoadRequest()
{
}

SQLLoadRequest::~SQLLoadRequest()
{
}

bool SQLLoadRequest::execute()
{
    logger().debug("[SQLLoadRequest] execute");
    std::ostringstream oss;

    if (_parameters.empty()) {
        CogServer& cogserver = static_cast<CogServer&>(server());
        PersistModule* persist =
            static_cast<PersistModule*>(cogserver.getModule(PersistModule::id()));
        AtomStorage* store = persist->getStore();
        if (store == NULL) oss << "error: invalid SQL storage" << std::endl;
        else store->load(const_cast<AtomTable&>(cogserver.getAtomSpace()->getAtomTable()));
    } else oss << info().help << std::endl;

    if (_mimeType == "text/plain")
        send(oss.str());

    return true;
}

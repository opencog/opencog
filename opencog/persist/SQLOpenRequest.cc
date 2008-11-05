/*
 * opencog/persist/SQLOpenRequest.cc
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

#include <sstream>

#include <opencog/persist/AtomStorage.h>
#include <opencog/persist/PersistModule.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

using namespace opencog;

bool SQLOpenRequest::execute()
{
    logger().debug("[SQLOpenRequest] execute");
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

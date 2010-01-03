/*
 * opencog/server/ServerRequestWrapper.cc
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

#include "ServerRequestWrapper.h"

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>

using namespace opencog;

ServerRequestWrapper::ServerRequestWrapper()
{
}

ServerRequestWrapper::~ServerRequestWrapper()
{
}

void ServerRequestWrapper::handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::ostringstream oss;
    CogServer& cogserver = static_cast<CogServer&>(server());
    boost::regex reg("request/([^/]*)");
    boost::cmatch m;

    if (boost::regex_search(ri->uri,m,reg)) {
        std::string requestName(m[1].first, m[1].second);
        std::list<const char*> commands = cogserver.requestIds();
        Request* request = cogserver.createRequest(requestName.c_str());
        if (request == NULL) {
            mg_printf(conn, "unknown request %s\n", requestName.c_str());
            return;
        }
        /// HMMMMM - seems we need to make a fake ConsoleSocket to pass
        /// to the Requests otherwise we can't get their output
    } else {
        mg_printf(conn, "URL malformed? %s\n", ri->uri);
    }


    mg_printf(conn, "%s\n", ri->uri);
    mg_printf(conn, "%s\n", ri->query_string);
    mg_printf(conn, "%d\n", ri->post_data_len);
    mg_printf(conn, "%s\n", ri->post_data);
    //   not found

    //
    //const RequestClassInfo& cci = cogserver.requestInfo(_parameters.front());
    //    if (cci.help != "")
    //        oss << cci.help << std::endl;

}


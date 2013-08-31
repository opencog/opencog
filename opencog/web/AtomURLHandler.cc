/*
 * opencog/rest/AtomURLHandler.cc
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * Copyright (C) 2010 by Joel Pitt
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#include "AtomURLHandler.h"

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <opencog/atomspace/Handle.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>

/**
  * @todo When/if we upgrade to Boost >= 1.41 replace json spirit with
  * boost::property_tree
  */
#include <opencog/web/json_spirit/json_spirit.h>

#include "WebModule.h"

#include "GetAtomRequest.h"

using namespace opencog;
using namespace json_spirit;

AtomURLHandler::AtomURLHandler() : BaseURLHandler("text/plain"),isJSON(false),
    refreshPage(false), handleInQuery(false), handleInURL(false),
    h(Handle::UNDEFINED)
{
}

AtomURLHandler::~AtomURLHandler()
{
}

void AtomURLHandler::handlePOSTCreate( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = BaseURLHandler::splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());

    std::string json_str;
    if (ri->post_data_len > 0) {
        json_str = std::string(ri->post_data, ri->post_data_len);
        std::cout << "#params" << params.size() << "data:" << json_str << std::endl;
        params.push_back(json_str);
    } else {
        mg_printf(_conn, "{\"error\":\"no_data\"}");
        completed = true;
        return;
    }

    Request* request = cogserver.createRequest("json-create-atom");
    if (request == NULL) {
        WebModule::return500( conn, std::string("unknown request"));
        completed = true;
        return;
    }
    request->setRequestResult(this);
    request->setParameters(params);
    cogserver.pushRequest(request);
}

void AtomURLHandler::handlePOSTUpdate( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::string handleUUID;
    std::list<std::string> params = BaseURLHandler::splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());

    // check that no other parameters are given!
    if (params.size() > 0) {
        // Check that the only query string option is a handle
        if (!handleInQuery || params.size() > 1) {
            mg_printf(_conn, "{\"error\":\"unknown query string\"}");
            completed = true;
            return;
        }
    } else {
        std::string handleParam;
        handleParam = "handle=" + h;
        params.push_front(handleParam);
    }

    std::string json_str;
    if (ri->post_data_len > 0) {
        json_str = std::string(ri->post_data, ri->post_data_len);
        params.push_back(json_str);
    } else {
        mg_printf(_conn, "{\"error\":\"no_data\"}");
        completed = true;
        return;
    }

    // TODO create update request
    Request* request = cogserver.createRequest("json-update-atom");
    if (request == NULL) {
        WebModule::return500( conn, std::string("unknown request"));
        completed = true;
        return;
    }
    request->setRequestResult(this);
    request->setParameters(params);
    cogserver.pushRequest(request);
}

void AtomURLHandler::handlePUT( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    WebModule::return405( conn);
    completed = true;
    return;
}

void AtomURLHandler::handleDELETE( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    // TODO implement me
    WebModule::return405( conn);
    completed = true;
    return;
}

void AtomURLHandler::handleGET( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = BaseURLHandler::splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());
    Request* request = cogserver.createRequest("json-get-atom");
    if (request == NULL) {
        WebModule::return500( conn, std::string("unknown request"));
        completed = true;
        return;
    }
    // If we are passed data, then this is a json request
    if (data) {
        isJSON = true;
        params.push_back(std::string("format=json"));
    }
    else {
        // Check for refresh option
        //! @todo make refresh time specifiable on the page.
        std::list<std::string>::const_iterator it;
        for (it = params.begin(); it != params.end(); ++it) {
            if (*it == "refresh=1" or *it == "refresh=true") {
                refreshPage = true;
                break;
            }
        }
    }
    // Take handle UUID from URL if it exists and create parameter
    if (handleInURL && !handleInQuery) {
        std::string handleUUID;
        handleUUID = "handle=" + h;
        params.push_back(handleUUID);
    }

    request->setRequestResult(this);
    request->setParameters(params);
    cogserver.pushRequest(request);
}

void AtomURLHandler::handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    _conn = conn;
    call_url = ri->uri;
    if (ri->query_string) query_string = ri->query_string;

    // Check if this request is to a specific handle
    boost::regex atomInURL("atom/([^/]+)");
    boost::regex atomInQuery("handle=([^&]+)");
    boost::cmatch m1, m2;
    UUID uuid;
    if (boost::regex_search(ri->uri,m1,atomInURL)) {
        std::string handleStr(m1[1].first, m1[1].second);
        try {
            uuid = boost::lexical_cast<UUID>(handleStr);
        } catch (const boost::bad_lexical_cast&) {
            mg_printf(_conn, "{\"error\":\"error parsing handle %s in URL\"}\n",
                    handleStr.c_str());
            completed = true;
            return;
        }
        h = Handle(uuid);
        handleInURL = true;
    } else if (boost::regex_search(query_string.c_str(),m2,atomInQuery)) {// Search query string too
        std::string handleStr(m2[1].first, m2[1].second);
        try {
            uuid = boost::lexical_cast<UUID>(handleStr);
        } catch (const boost::bad_lexical_cast&) {
            mg_printf(_conn, "{\"error\":\"error parsing handle %s in query string\"}\n",
                    handleStr.c_str());
            completed = true;
            return;
        }
        h = Handle(uuid);
        handleInQuery = true;
    }

    method = ri->request_method;
    if (method == "GET")
        handleGET(conn, ri, data);
    else if (method == "POST") {
        if (handleInQuery || handleInURL) 
            handlePOSTUpdate(conn, ri, data);
        else
            handlePOSTCreate(conn, ri, data);
    } else if (method == "PUT") {
        // Does nothing but returns unsupported method
        handlePUT(conn, ri, data);
    } else if (method == "DELETE") {
        // TODO support deletion of atoms
        if (handleInQuery || handleInURL) 
            handleDELETE(conn, ri, data);

    }

}

std::string AtomURLHandler::getHTMLHeader()
{
    std::ostringstream oss;
    oss << "<script language=\"javascript\" src=\"/resources/processing-0.9.7.js\">"
        "</script>" << std::endl;
    oss << "<script language=\"javascript\" src=\"/resources/jquery-1.4.2.js\">"
        "</script>" << std::endl;
    oss << "<script language=\"javascript\">"
        "handle=\"" << h << "\";"
        "</script>" << std::endl;
    //oss << "<script language=\"javascript\" src=\"../resources/init.js\">"
        //"</script>" << std::endl;
    return oss.str();
}

void AtomURLHandler::OnRequestComplete()
{
    if (method == "GET") {
        //! @todo replace with configured server
        std::stringstream result;
        std::string serverAdd("http://localhost:17034");
        serverAdd += UI_PATH_PREFIX;
        
        if (!isJSON) {
            result << WebModule::openHtmlHeader();
            if (refreshPage)
                 result << WebModule::HtmlrefreshHeader();
            result << getHTMLHeader();
             result << WebModule::closeHtmlHeader();
        } else {
            result << WebModule::jsonHeader();
        }

        result << replaceURL(serverAdd);

	mg_printf(_conn, result.str().c_str());

        if (!isJSON) {
            result.str("");
            result << "\r\n\r\n<small>You requested the url: %s<br> With query string:"
                "%s</small>";
            if (refreshPage) {
                result << "<br/><small>Page will refresh every 5 seconds</small>";
            }
            result << WebModule::htmlFooter();
            mg_printf(_conn, result.str().c_str(), call_url.c_str(),
                    query_string.c_str());
	}
    } else if (method == "POST") {
        mg_printf(_conn, request_output.str().c_str());
    }
    completed = true;

}


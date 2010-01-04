/*
 * opencog/rest/RESTModule.cc
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
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

#include "RESTModule.h"

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>
#include <opencog/util/Config.h>

#include <sstream>
#include <boost/bind.hpp>
#include <boost/mem_fn.hpp>
#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

using namespace opencog;

RESTModule *rest_mod;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()   { return RESTModule::id(); }
extern "C" Module*     opencog_module_load() {
    rest_mod = new RESTModule();
    return rest_mod;
}
extern "C" void        opencog_module_unload(Module* module) { delete module; }

const char* RESTModule::DEFAULT_SERVER_ADDRESS = "http://localhost";

//! @todo create a function to generate header
const char* RESTModule::open_html_header = "HTTP/1.1 200 OK\r\n"
    "content-Type: text/html\r\n\r\n"
    "<html><head>";
const char* RESTModule::close_html_header = "</head><body>";

const char* RESTModule::html_refresh_header = 
    "<META HTTP-EQUIV=\"Refresh\" CONTENT=\"5\">";

const char* RESTModule::html_footer = "</body></html>\r\n";

RESTModule::RESTModule() : _port(DEFAULT_PORT), serverAddress(DEFAULT_SERVER_ADDRESS)
{
    logger().debug("[RESTModule] constructor");

    if (config().has("REST_PORT"))
        _port = config().get_int("REST_PORT");
    if (config().has("REST_SERVER"))
        serverAddress = config().get("REST_SERVER");

    // Register all requests with CogServer
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerRequest(GetAtomRequest::info().id, &getAtomFactory); 
    cogserver.registerRequest(GetListRequest::info().id, &getListFactory); 

    timeout = 100;

}

RESTModule::~RESTModule()
{
    rest_mod = NULL;
    logger().debug("[RESTModule] destructor");
    //CogServer& cogserver = static_cast<CogServer&>(server());
    mg_stop(ctx);
}

void RESTModule::init()
{
    logger().debug("[RESTModule] init");
    // Set the port that the embedded mongoose webserver will listen on.
    std::stringstream port_str;
    port_str << _port;
    ctx = mg_start();
    mg_set_option(ctx, "ports", port_str.str().c_str());
    // Set up the urls
    setupURIs();
}

void viewAtomPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
void viewListPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
void makeRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);

void RESTModule::setupURIs()
{
    // Support both "atom/UUID" and "atom?handle=UUID"
    mg_set_uri_callback(ctx, PATH_PREFIX "/atom/*", viewAtomPage, NULL);
    mg_set_uri_callback(ctx, PATH_PREFIX "/atom", viewAtomPage, NULL);
    mg_set_uri_callback(ctx, "/atom", viewAtomPage, NULL);
    mg_set_uri_callback(ctx, PATH_PREFIX "/list", viewListPage, NULL);
    mg_set_uri_callback(ctx, PATH_PREFIX "/list/*", viewListPage, NULL);
    mg_set_uri_callback(ctx, PATH_PREFIX "/request/*", makeRequest, NULL);
}

void RESTModule::return400(mg_connection* conn, const std::string& message)
{
    mg_printf(conn, "HTTP/1.1 400 %s\r\n", message.c_str());
}

void RESTModule::return404(mg_connection* conn)
{
    mg_printf(conn, "HTTP/1.1 404 Not found.\r\n");
}

void RESTModule::return500(mg_connection* conn, const std::string& message)
{
    mg_printf(conn, "HTTP/1.1 500 %s\r\n", message.c_str());
}

std::list<std::string> splitQueryString(char* query) {
    using namespace std;
    list<string> params;
    if (query == NULL) return params;
    string query_string(query);
    boost::split(params, query_string, boost::is_any_of("&;"));
    return params;
}

std::map<std::string,std::string> paramsToMap(const std::list<std::string>& params) 
{
    using namespace std;
    map<string,string> query_map;
    list<string>::const_iterator p_i;
    for (p_i = params.begin(); p_i != params.end(); p_i++) {
        std::vector<string> keyAndValue;
        boost::split(keyAndValue, *p_i, boost::is_any_of("="));
        if (keyAndValue.size() != 2) continue;
        query_map.insert(pair<string,string>(keyAndValue[0], keyAndValue[1]));
    }
    return query_map;
}

void viewAtomPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());

    // Get handle UUID from URL if it exists
    boost::regex reg("atom/([^/]*)");
    boost::cmatch m;
    if (boost::regex_search(ri->uri,m,reg)) {
        std::string handleUUID(m[1].first, m[1].second);
        handleUUID = "handle=" + handleUUID;
        params.push_back(handleUUID);
    }

    Request* request = cogserver.createRequest("get-atom");
    if (request == NULL) {
        RESTModule::return500( conn, std::string("unknown request"));
        return;
    }
    // Prevent CogServer from deleting this request
    // after it executes so we can find details about it.
    request->cleanUp = false;
    request->setParameters(params);
    cogserver.pushRequest(request);
    
    for (;;) {
        //! @todo - implement time out (requires lock on the request cleanUp variable
        if (request->complete) break;
    }
    GetAtomRequest *gar = dynamic_cast<GetAtomRequest *>(request);

    //! @todo replace with configured server
    std::stringstream result;
    std::string serverAdd("http://localhost:17034");
    serverAdd += PATH_PREFIX;
    
    // Check for refresh option
    //! @todo make refresh time specifiable on the page.
    bool refresh = false;
    std::list<std::string>::const_iterator it;
    for (it = params.begin(); it != params.end(); ++it) {
        if (*it == "refresh=1" or *it == "refresh=true") {
            refresh = true;
            break;
        }
    }
    result << RESTModule::open_html_header;
    if (refresh)
        result << RESTModule::html_refresh_header;
    result << gar->getHTMLHeader();
    result << RESTModule::close_html_header;

    result << gar->getHTML(serverAdd).c_str();
//mg_printf(conn, result.str().c_str());
    char buffer[512];
    for (uint i = 0; i < result.str().size(); i+=511) {
        memset(buffer,'\0',512);
        result.str().copy(buffer,511,i);
        if (i+511 > result.str().size())
            buffer[(result.str().size() % 511) + 1] = '\0';
        mg_printf(conn, buffer);
    }
        
    result.str("");
    result << "\r\n\r\n<small>You requested the url: %s<br> With query string:"
        "%s</small>";
    if (refresh) {
        result << "<br/><small>Page will refresh every 5 seconds</small>";
    }
    result << RESTModule::html_footer;
    mg_printf(conn, result.str().c_str(), ri->uri, ri->query_string);

    // Clean up
    delete request;
    
}

void viewListPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());
    Request* request = cogserver.createRequest("get-list");
    if (request == NULL) {
        RESTModule::return500( conn, std::string("unknown request"));
        return;
    }
    // Get list parameters from URL if they exist
    boost::regex reg("list/([^/]*)");
    boost::cmatch m;
    if (boost::regex_search(ri->uri,m,reg)) {
        std::string typeName(m[1].first, m[1].second);
        typeName = "type=" + typeName;
        params.push_back(typeName);
    }

    // Prevent CogServer from deleting this request
    // after it executes so we can find details about it.
    request->cleanUp = false;
    request->setParameters(params);
    cogserver.pushRequest(request);
    
    for (;;) {
        //! @todo - implement time out (requires lock on the request cleanUp variable
        if (request->complete) break;
    }
    GetListRequest *glr = dynamic_cast<GetListRequest *>(request);

    std::stringstream result;
    std::string serverAdd("http://localhost:17034");
    serverAdd += PATH_PREFIX;
    
    // Check for refresh option
    //! @todo make refresh time specifiable on the page.
    bool refresh = false;
    std::list<std::string>::const_iterator it;
    for (it = params.begin(); it != params.end(); ++it) {
        if (*it == "refresh=1" or *it == "refresh=true") {
            refresh = true;
            break;
        }
    }
    result << RESTModule::open_html_header;
    if (refresh)
        result << RESTModule::html_refresh_header;
    result << RESTModule::close_html_header;

    result << glr->getHTML(serverAdd).c_str();
//mg_printf(conn, result.str().c_str());
    char buffer[512];
    for (uint i = 0; i < result.str().size(); i+=511) {
        memset(buffer,'\0',512);
        result.str().copy(buffer,511,i);
        if (i+511 > result.str().size())
            buffer[(result.str().size() % 511) + 1] = '\0';
        mg_printf(conn, buffer);
    }
        
    result.str("");
    result << "\r\n\r\n<small>You requested the url: %s<br/> With query string:"
        "%s</small>";
    if (refresh) {
        result << "<br/><small>Page will refresh every 5 seconds</small>";
    }
    result << RESTModule::html_footer;
    mg_printf(conn, result.str().c_str(), ri->uri, ri->query_string);

    // Clean up
    delete request;
    
}

void makeRequest ( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    rest_mod->requestWrapper.handleRequest(conn, ri, data);
}

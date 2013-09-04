/*
 * opencog/rest/WebModule.cc
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

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/mem_fn.hpp>

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>
#include <opencog/util/Config.h>

#include "BaseURLHandler.h"
#include "WebModule.h"

using namespace opencog;


DECLARE_MODULE(WebModule)

const char* WebModule::DEFAULT_SERVER_ADDRESS = "http://localhost";

WebModule::WebModule(CogServer& cs) :
    Module(cs), _port(DEFAULT_PORT), serverAddress(DEFAULT_SERVER_ADDRESS)
{
    if (config().has("Web_PORT"))
        _port = config().get_int("Web_PORT");
    if (config().has("Web_SERVER"))
        serverAddress = config().get("Web_SERVER");

    // Register all requests with CogServer
    _cogserver.registerRequest(GetAtomRequest::info().id, &getAtomFactory);
    _cogserver.registerRequest(GetListRequest::info().id, &getListFactory);
    _cogserver.registerRequest(CreateAtomRequest::info().id, &createAtomFactory);
    _cogserver.registerRequest(UpdateAtomRequest::info().id, &updateAtomFactory);

    timeout = 100;
}

WebModule::~WebModule()
{
    // Stop mongoose webserver
    mg_stop(ctx);
}

static const char* DEFAULT_WEB_PATH[] =
{
    "../opencog/web", // For running from bin dir that's in root of src
    DATADIR"/www",
#ifndef WIN32
    "/usr/share/opencog/www",
    "/usr/local/share/opencog/www",
#endif // !WIN32
    NULL
};

void WebModule::init()
{
    logger().debug("[WebModule] init");
    // Set the port that the embedded mongoose webserver will listen on.
    std::stringstream port_str;
    port_str << _port;
    ctx = mg_start();
    mg_set_option(ctx, "ports", port_str.str().c_str());
    // Turn on admin page
    //mg_set_option(ctx, "admin_uri", "/admin/");
    // Find and then set path for web resource files
    int i = 0;
    for (; DEFAULT_WEB_PATH[i] != NULL; ++i) {
        boost::filesystem::path webPath(DEFAULT_WEB_PATH[i]);
        webPath /= "resources/processing.js";
        if (boost::filesystem::exists(webPath)) {
            //std::cout << webPath << std::endl;
            break;
        }
    }
    if (DEFAULT_WEB_PATH[i] == NULL) {
        logger().error("[WebModule] Can't find web resources dir.");
    } else {
        logger().fine("[WebModule] Using resources directory %s", DEFAULT_WEB_PATH[i]);
        mg_set_option(ctx, "root", DEFAULT_WEB_PATH[i]);
    }
    // Turn off directory listing
    mg_set_option(ctx, "dir_list", "no");
    // Set up the urls
    setupURIs();
}

void viewAtomPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
void viewListPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
void makeRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);

void WebModule::setupURIs()
{
    setupURIsForREST();
    setupURIsForUI();
}

void WebModule::setupURIsForUI()
{
    // Support both "atom/UUID" and "atom?handle=UUID"
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/atom/*", viewAtomPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/atom", viewAtomPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/list/*", viewListPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/list", viewListPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/server/request/*", makeRequest, NULL);
}

void WebModule::setupURIsForREST()
{
    static char rest_str[] = "rest";
    // atom/type/* support GET atoms of type.
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/list/*", viewListPage,
            rest_str);
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/list", viewListPage,
            rest_str);
    // atom/ support GET/PUT/POST == get info/create/create
    // atom/* support GET, get atom info
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/atom/*", viewAtomPage,
            rest_str);
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/atom", viewAtomPage,
            rest_str);
    // server/request/request_name, POST
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/server/request/*", makeRequest,
            rest_str);
}

void WebModule::return400(mg_connection* conn, const std::string& message)
{
    mg_printf(conn, "HTTP/1.1 400 %s\r\n", message.c_str());
}

void WebModule::return404(mg_connection* conn)
{
    mg_printf(conn, "HTTP/1.1 404 Not found.\r\n");
}

void WebModule::return405(mg_connection* conn)
{
    mg_printf(conn, "HTTP/1.1 404 Method Not Allowed.\r\n");
}

void WebModule::return500(mg_connection* conn, const std::string& message)
{
    mg_printf(conn, "HTTP/1.1 500 %s\r\n", message.c_str());
}

void viewAtomPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    AtomURLHandler *handler = new AtomURLHandler();
    handler->handleRequest(conn, ri, data);
    while (!handler->completed) {usleep(50);};
    delete handler;
}

void viewListPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    ListURLHandler *handler = new ListURLHandler();
    handler->handleRequest(conn, ri, data);
    while (!handler->completed) {usleep(50);};
    delete handler;
}

void makeRequest ( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    ServerRequestWrapper *handler = new ServerRequestWrapper();
    handler->handleRequest(conn, ri, data);
    while (!handler->completed) {usleep(50);};
    delete handler;
}



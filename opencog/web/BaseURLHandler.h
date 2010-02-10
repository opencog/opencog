/*
 * opencog/rest/BaseURLHandler.h
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

#ifndef _OPENCOG_BASE_URL_HANDLER_H
#define _OPENCOG_BASE_URL_HANDLER_H

#include <vector>
#include <string>
#include <map>
#include <list>
#include <sstream>

#include <opencog/server/RequestResult.h>

#include "mongoose.h"

#define SERVER_PLACEHOLDER "REST_SERVER_ADDRESS"
namespace opencog
{

class BaseURLHandler : public RequestResult
{

protected:
    std::ostringstream request_output;
    struct mg_connection *_conn;
public:

    BaseURLHandler(const std::string& mimeType) : RequestResult(mimeType), completed(false) {};
    ~BaseURLHandler() {};
    virtual void handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data) = 0;

    static std::list<std::string> splitQueryString(char* query);
    static std::map<std::string,std::string> paramsToMap (
            const std::list<std::string>& params);
    std::string replaceURL(const std::string server_string);

    // Interface for RequestResult follows:

    // Only needed by very specific requests:
    virtual void SetDataRequest() {};
    virtual void Exit() {};

    /** receive data from a Request */
    virtual void SendResult(const std::string& res);
    /** called when a Request has finished. */
    virtual void OnRequestComplete() = 0;
    // ----
    bool completed;

};

} // namespace 

#endif // _OPENCOG_BASE_URL_HANDLER_H

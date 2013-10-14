/*
 * opencog/rest/ListURLHandler.h
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

#ifndef _OPENCOG_LIST_URL_HANDLER_H
#define _OPENCOG_LIST_URL_HANDLER_H

#include <vector>
#include <string>

#include <opencog/server/Request.h>
#include <opencog/server/RequestResult.h>

#include "BaseURLHandler.h"
#include "GetListRequest.h"

#include "mongoose.h"

namespace opencog
{

class ListURLHandler: public BaseURLHandler
{

    bool refreshPage;
    bool isJSON;
    std::string call_url;
    std::string query_string;

public:

    ListURLHandler();
    ~ListURLHandler();
    void handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);

    virtual void OnRequestComplete();

};

} // namespace 

#endif // _OPENCOG_LIST_URL_HANDLER_H

/*
 * opencog/rest/GetListRequest.h
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

#ifndef _OPENCOG_GET_LIST_REQUEST_H
#define _OPENCOG_GET_LIST_REQUEST_H

#include <sstream>
#include <string>
#include <vector>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/server/Request.h>
#include <opencog/server/RequestClassInfo.h>

namespace opencog
{

#define GETLIST_MAXIMUM_RESULTS 50
class GetListRequest : public Request
{

protected:
    enum { json_format, html_tabular_format } output_format;

    std::ostringstream  _output;
    std::string order_by;
    bool descending;
    std::string name;
    Type type;
    bool subtypes;
    HandleSeq requestHandles;
    int maximum, skip;

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "json-get-list",
            "Get details for a particular atom referred by handle",
            "Usage: json-get-list handle=<handle> name=<name> type=<type>\n\n"
            "Get details for a particular atom referred by handle\n"
            "   <handle>: list the atom referred by handle (multi-use)\n"
            "   <name>:   name of atom\n"
            "   <type>:   type of atom\n",
            false, true // not shell, is hidden
        );
        return _cci;
    }

    GetListRequest(CogServer&);
    virtual ~GetListRequest();
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
    bool sortHandles(HandleSeq& hs, std::string order_by, bool descend = true);

    void html_makeListHeader(unsigned int total_results);
    void html_makeOutput(HandleSeq& hs);

    void json_makeOutput(HandleSeq& hs);

    void sendError(std::string s);

    //std::string getHTML(std::string);
};


} // namespace 

#endif // _OPENCOG_LIST_REQUEST_H

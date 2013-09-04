/*
 * opencog/rest/CreateAtomRequest.h
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

#ifndef _OPENCOG_CREATE_ATOM_REQUEST_H
#define _OPENCOG_CREATE_ATOM_REQUEST_H

#include <sstream>
#include <string>
#include <vector>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/server/Request.h>
#include <opencog/server/RequestClassInfo.h>

#include <opencog/web/json_spirit/json_spirit.h>

namespace opencog
{

class CreateAtomRequest : public Request
{

protected:

    std::ostringstream  _output;

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "json-create-atom",
            "Create a new atom from JSON",
            "Usage: json-create-atom<CRLF>JSON<CRLF><Ctrl-D><CRLF>\n\n"
            "   Create an atom based on the JSON format:\n"
            "   { \"type\": TYPENAME, \"name\": NAME, \n"
            "     \"outgoing\": [ UUID1, UUID2 ... ], \n"
            "     \"truthvalue\": {\"simple|composite|count|indefinite\":\n"
            "          [truthvalue details] } \n",
            true, false
        );
        return _cci;
    }

    CreateAtomRequest(CogServer&);
    virtual ~CreateAtomRequest();
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
    void json_makeOutput(Handle h, bool exists);
    void generateProcessingGraph(Handle h);
    void setRequestResult(RequestResult* rr);
    void decode(std::string &str); 
};

} // namespace 

#endif // _OPENCOG_CREATE_ATOM_REQUEST_H

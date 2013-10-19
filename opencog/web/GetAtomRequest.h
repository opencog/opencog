/*
 * opencog/rest/GetAtomRequest.h
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

#ifndef _OPENCOG_GET_ATOM_REQUEST_H
#define _OPENCOG_GET_ATOM_REQUEST_H

#include <sstream>
#include <string>
#include <vector>

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/server/Request.h>
#include <opencog/server/RequestClassInfo.h>

namespace opencog
{

class GetAtomRequest : public Request
{

protected:

    enum { json_format, html_tabular_format } output_format;
    std::ostringstream  _output;

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "json-get-atom",
            "Get details for a particular atom referred by handle",
            "Usage: json-get-atom handle=<handle>\n\n"
            "Get details for a particular atom referred by handle\n"
            "   <handle>: list the atom identified by the specified handle\n",
            false, true // not shell, is hidden
        );
        return _cci;
    }

    GetAtomRequest(CogServer&);
    virtual ~GetAtomRequest();
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
    std::string getHTML(std::string);
    std::string getHTMLHeader();
    void html_makeOutput(Handle);
    static std::string json_makeOutput(CogServer&, Handle);
    static std::string tvToJSON(TruthValuePtr);
    void generateProcessingGraph(Handle);
};

} // namespace 

#endif // _OPENCOG_GET_ATOM_REQUEST_H

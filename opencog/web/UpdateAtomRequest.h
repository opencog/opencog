/*
 * opencog/rest/UpdateAtomRequest.h
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

#ifndef _OPENCOG_UPDATE_ATOM_REQUEST_H
#define _OPENCOG_UPDATE_ATOM_REQUEST_H

#include <sstream>
#include <string>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/server/Request.h>
#include <opencog/server/RequestClassInfo.h>

#include <opencog/web/json_spirit/json_spirit.h>

namespace opencog
{

class UpdateAtomRequest : public Request
{

protected:

    std::ostringstream  _output;

    enum {sti_none, sti_replace, sti_add, sti_subtract, sti_merge} sti_mod;
    enum {lti_none, lti_replace, lti_add, lti_subtract, lti_merge} lti_mod;
    enum {tv_none, tv_replace, tv_merge} tv_mod;

    bool doSTIChanges(AtomSpace* as, Handle h, AttentionValue::sti_t sti_x );
    bool doLTIChanges(AtomSpace* as, Handle h, AttentionValue::lti_t lti_x );
    bool doTVChanges(AtomSpace* as, Handle h, TruthValue* tv);

    TruthValue *tv;
public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "json-update-atom",
            "update the STI, LTI, or TV of an atom with JSON message",
            "Usage: json-update-atom handle {JSON}\n\n"
            "   Update an atom based on a JSON message with format:\n"
            "   { \"sti\": STI } \n"
            "   { \"lti\": LTI } \n"
            "   { \"tv\": {tv details} } \n",
            true, false 
        );
        return _cci;
    }

    UpdateAtomRequest(CogServer&);
    virtual ~UpdateAtomRequest();
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
    void json_makeOutput(Handle h, bool exists);
    void generateProcessingGraph(Handle h);
    void setRequestResult(RequestResult* rr);
    void decode(std::string &str); 
};

} // namespace 

#endif // _OPENCOG_UPDATE_ATOM_REQUEST_H


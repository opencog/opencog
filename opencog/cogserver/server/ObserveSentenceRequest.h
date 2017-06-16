/*
 * opencog/cogserver/server/ObserveSentenceRequest.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Curtis Faith
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

#ifndef _OPENCOG_OBSERVE_SENTENCE_REQUEST_H
#define _OPENCOG_OBSERVE_SENTENCE_REQUEST_H

#include <sstream>
#include <string>
#include <vector>
#include <thread>

#include <opencog/atoms/base/Handle.h>
#include <opencog/cogserver/server/Request.h>
#include <opencog/cogserver/server/RequestClassInfo.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class ObserveSentenceRequest : public Request
{

protected:

    std::string _sentence;
    std::ostringstream  _error;

    void sendOutput(void);
    void sendError (void);
    bool syntaxError(void);

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "observe",
            "Observe the sentence and create corresponding language learning atoms",
            "Usage: learn [-pair_distance <limit>] \"<sentence-in-quotes>\"\n\n"
            "Create language learning atoms for the sentence. Optional flags are:\n"
            "    -pair_distance <limit>    Create pairs up to <limit> distance apart.\n"
            "    -quiet                    Do not return status over telnet.\n"
            "    -noop                     Perform no op-erations (useful for timing).\n"
        );
        return _cci;
    }

    ObserveSentenceRequest(CogServer&);
    virtual ~ObserveSentenceRequest();
    virtual bool isSynchronous(void) { return true; }
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
};

/** @}*/
} // namespace 

#endif // _OPENCOG_OBSERVE_SENTENCE_REQUEST_H

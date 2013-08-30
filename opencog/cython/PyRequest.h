/*
 * opencog/cython/PyRequest.h
 *
 * Copyright (C) 2011 by The OpenCog Foundation
 * All Rights Reserved
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

#ifndef _OPENCOG_PYREQUEST_H
#define _OPENCOG_PYREQUEST_H

#include <string>

#include "PyIncludeWrapper.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Request.h>
#include <opencog/server/RequestClassInfo.h>

namespace opencog
{

class PyRequest : public Request
{

protected:
    PyObject* _pyrequest;
    std::string _moduleName;
    std::string _className;
    std::string _last_result;
    RequestClassInfo* _cci;

public:

    const RequestClassInfo& info() const { return *_cci; }

    /** Request's constructor */
    PyRequest(CogServer&, const std::string& moduleName, const std::string& className,
              RequestClassInfo*);

    /** Request's desconstructor */
    virtual ~PyRequest();

    /**  Returns 'true' if the command completed successfully and 'false' otherwise. */
    virtual bool execute(void);

    virtual bool isShell(void) { return info().is_shell; }

};

} // namespace 

#endif // _OPENCOG_PYREQUEST_H

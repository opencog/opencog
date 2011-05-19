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

#include <Python.h>

#include <opencog/server/Request.h>
#include <opencog/server/Factory.h>
#include <opencog/atomspace/AtomSpace.h>

class PythonModuleUTest;

namespace opencog
{

class PyRequest : public Request
{

protected:
    PyObject* pyrequest;
    std::string moduleName;
    std::string className;
    std::string last_result;
    RequestClassInfo* cci;

public:

    const RequestClassInfo& info() const {
        return *cci;
    }

    /** Request's constructor */
    PyRequest(const std::string& _moduleName, const std::string& _className);

    /** Request's desconstructor */
    virtual ~PyRequest();

    /**  Returns 'true' if the command completed successfully and 'false' otherwise. */
    virtual bool execute(void);

    /** Not a shell */
    virtual bool isShell(void) { return false; }

};

} // namespace 

#endif // _OPENCOG_PYREQUEST_H

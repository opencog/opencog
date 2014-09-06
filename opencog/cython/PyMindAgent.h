/*
 * opencog/cython/PyMindAgent.h
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

#ifndef _OPENCOG_PYAGENT_H
#define _OPENCOG_PYAGENT_H

#include <string>

#include "PyIncludeWrapper.h"

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>

class PythonModuleUTest;

namespace opencog
{

/** The PyMindAgent Class
 * This class wraps Python MindAgents and allows the CogServer to interact
 * and manage them.
 */
class PyMindAgent : public Agent
{
    friend class ::PythonModuleUTest;

    PyObject* pyagent;
    std::string moduleName;
    std::string className;

public:

    virtual const ClassInfo& classinfo() const;

    //PyMindAgent();

    /** Pass a PyObject that is a Python MindAgent object.  */
    PyMindAgent(CogServer&, const std::string& moduleName, const std::string& className);

    virtual ~PyMindAgent();

    /** Run method - this calls the run method of the encapsulated Python
     * MindAgent class.
     */
    virtual void run();

}; // class

typedef std::shared_ptr<PyMindAgent> PyMindAgentPtr;

}  // namespace

#endif // _OPENCOG_AGENT_H


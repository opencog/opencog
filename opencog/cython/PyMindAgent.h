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

#include <Python.h>

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>

namespace opencog
{

class CogServer;
class AtomSpaceImpl;

/** The PyMindAgent Class
 * This class wraps Python MindAgents and allows the CogServer to interact
 * and manage them.
 */
class PyMindAgent : public Agent
{

protected:

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::PyMindAgent");
        return _ci;
    }


    PyMindAgent();

    /** Agent's constructor. Pass a PyObject that is a Python MindAgent object.
     */
    PyMindAgent(PyObject* py_agent, std::string& moduleFileName);

    /** Agent's destructor */
    virtual ~PyMindAgent();

    /** Abstract run method. Should be overriden by a derived agent with the
     *  actual agent's behavior. */
    virtual void run(CogServer* server);


}; // class

}  // namespace

#endif // _OPENCOG_AGENT_H


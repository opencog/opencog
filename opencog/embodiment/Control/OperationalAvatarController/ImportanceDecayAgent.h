/*
 * opencog/embodiment/Control/OperationalAvatarController/ImportanceDecayAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef IMPORTANCEDECAYAGENT_H
#define IMPORTANCEDECAYAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>
#include <time.h>

namespace opencog { namespace oac {

class ImportanceDecayAgent : public opencog::Agent
{

private:

    time_t lastTickTime;
    /**
     * signal connections used to keep track of atom merge in the AtomSpace
     */
    boost::signals::connection mergedAtomConnection;

public:

    ImportanceDecayAgent(CogServer&);
    virtual ~ImportanceDecayAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::oac::ImportanceDecayAgent");
        return _ci;
    }

    virtual void run();

    // connects to the signals from AtomSpace it needs to know
    void connectSignals(AtomSpace& as);

    /**
     * Method to receive atom merge signals from AtomSpace
     */
    void atomMerged(AtomSpaceImpl* as, Handle h);

}; // class

} } // namespace opencog::oac

#endif

/*
 * opencog/embodiment/Control/MessagingSystem/ImportanceDecayAgent.h
 *
 * Copyright (C) 2007-2008 Andre Senna
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

#ifndef IMPORTANCEDECAYAGENT_H
#define IMPORTANCEDECAYAGENT_H

#include <opencog/server/Agent.h>
#include <time.h>

namespace OperationalPetController {

class ImportanceDecayAgent : public opencog::Agent {

    private:

        time_t lastTickTime;

    public:

        ImportanceDecayAgent();
        virtual ~ImportanceDecayAgent();

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("OperationalPetController::ImportanceDecayAgent");
            return _ci;
        }

        void run(opencog::CogServer *server);

}; // class
}  // namespace

#endif

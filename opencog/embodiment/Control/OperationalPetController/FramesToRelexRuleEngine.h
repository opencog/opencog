/*
 * opencog/embodiment/Control/OperationalPetController/FramesToRelexRuleEngine.h
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Fabricio Silva 
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

#ifndef FRAMESTORELEXRULEENGINE_H
#define FRAMESTORELEXRULEENGINE_H

#include <opencog/atomspace/HandleSet.h>
#include <opencog/atomspace/AtomSpace.h>
#include "OutputRelexColor.h"
#include "OutputRelexYesNo.h"
#include "OutputRelexPhysiological.h"
//#include "FramesToRelexRule.h"

#include <string>

namespace OperationalPetController
{
    class FramesToRelexRuleEngine
    {
    public:
        FramesToRelexRuleEngine( );
        ~FramesToRelexRuleEngine( );
        OutputRelex* resolve( std::set< std::string > pre_conditions );
        static FramesToRelexRuleEngine& instance(void);
    private:
        //std::vector<FramesToRelexRule*> rules;
        std::map< std::set<std::string>, OutputRelex*> rules;
        static FramesToRelexRuleEngine* singletonInstance;
    };
};

#endif // FRAMESTORELEXRULEENGINE_H

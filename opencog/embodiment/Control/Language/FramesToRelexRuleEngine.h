/*
 * opencog/embodiment/Control/OperationalAvatarController/FramesToRelexRuleEngine.h
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

#include <opencog/atomspace/AtomSpace.h>

#include "OutputRelex.h"

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>
#include <fstream>

namespace opencog { namespace oac {

class FramesToRelexRuleEngine
{
public:
    FramesToRelexRuleEngine( );
    ~FramesToRelexRuleEngine( );
    OutputRelex* resolve( const std::set< std::string >& pre_conditions );
    void loadRules( void );//load the rules from the default file
    void loadRules( const std::string& ruleFileName );//load the rules from a specific file
private:
    std::map< std::set<std::string>, OutputRelex*> rules;
};

} } // namespace opencog::oac

#endif // FRAMESTORELEXRULEENGINE_H

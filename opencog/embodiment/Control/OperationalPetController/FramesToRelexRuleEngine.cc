/*
 * opencog/embodiment/Control/OperationalPetController/FramesToRelexRuleEngine.cc
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

#include "FramesToRelexRuleEngine.h"

using namespace OperationalPetController;
using namespace opencog;

FramesToRelexRuleEngine* FramesToRelexRuleEngine::singletonInstance = 0; 

FramesToRelexRuleEngine::FramesToRelexRuleEngine( void ) 
{
    OutputRelex* color_output_relex = new OutputRelexColor();
    std::set< std::string > color_pre_conditions;
    color_pre_conditions.insert("Entity");
    color_pre_conditions.insert("Color");
    rules.insert( std::pair<std::set<std::string>, OutputRelex*>(color_pre_conditions, color_output_relex) );

    OutputRelex* yesno_output_relex = new OutputRelexYesNo();
    std::set< std::string > yesno_pre_conditions;
    yesno_pre_conditions.insert("Message");
    rules.insert( std::pair<std::set<std::string>, OutputRelex*>(yesno_pre_conditions, yesno_output_relex) );

    OutputRelex* physiological_output_relex = new OutputRelexPhysiological();
    std::set< std::string > physiological_pre_conditions;
    physiological_pre_conditions.insert("Experiencer");
    physiological_pre_conditions.insert("Attribute");
    physiological_pre_conditions.insert("Degree");
    physiological_pre_conditions.insert("Value");
    rules.insert( std::pair<std::set<std::string>, OutputRelex*>(physiological_pre_conditions, physiological_output_relex) );


}

FramesToRelexRuleEngine::~FramesToRelexRuleEngine( void )
{
}

OutputRelex* FramesToRelexRuleEngine::resolve( std::set< std::string > pre_conditions )
{
    if( rules.find(pre_conditions) != rules.end() ){
        return rules.find(pre_conditions)->second;
    }else{
        printf("No rule found\n");
    }

    return NULL;
}

FramesToRelexRuleEngine& FramesToRelexRuleEngine::instance() 
{
    if ( !singletonInstance ) {
        singletonInstance = new FramesToRelexRuleEngine( );
    } // if
    return *singletonInstance;
}

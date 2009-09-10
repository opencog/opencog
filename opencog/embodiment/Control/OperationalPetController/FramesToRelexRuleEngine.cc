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
    loadRules();
}

void FramesToRelexRuleEngine::loadRules ( void ) {
    //TODO change this in order to work with the dist and change the
    //makeDistribution script to copy the rules file
    loadRules("embodiment/Controle/OperationalPetController/frames2relex-rules.txt");
}

void FramesToRelexRuleEngine::loadRules( const char* rulesFileName ){
    std::string rule;
    std::ifstream rulesFile(rulesFileName);
    if (rulesFile.is_open())
    {
        while (! rulesFile.eof() )
        {
            getline (rulesFile,rule);
            std::cout << "Rule: " << rule << std::endl;

            std::map< std::string, unsigned int> frame_elements_count;

            std::vector<std::string> tokens;
            boost::split( tokens, rule, boost::is_any_of( "|" ) );

            if ( tokens.size( ) != 2 ) {
                //std::cerr << "Error: " << tokens.size( ) << std::endl;
                logger().error("FramesToRelexRuleEngine::%s - ERROR while reading thr Frames2Reles rules. The tokes expected was 2, but was found %d.",
                        __FUNCTION__,tokens.size( ));
                return;
            } 

            std::vector<std::string> preconditions;//temporary preconditions, used when parsing the rules
            std::set<std::string> preconditions_set;//the set used as rule key
            boost::split( preconditions, tokens[0], boost::is_any_of( ";" ) );

            //store the order that each frame appears in the rule, which will be
            //used when replacing the variables $1, $2, and so on by the real
            //values
            std::map<unsigned int, std::string> frames_order;

            //match the frame name in the first group and the frame elements in
            //the second group
            const boost::regex frame( "(#\\w+)\\s*\\(([^\\)]+)\\)");
            
            unsigned int frameCounter = 0;
            unsigned int i;
            //this loop counts how many frames of each type appear in the rule
            for( i = 0; i < preconditions.size( ); ++i ) {
                boost::cmatch matches;
                if ( boost::regex_match( preconditions[i].c_str( ), matches, frame ) ) {
                    std::vector<std::string> elements;
                    std::string frameName = matches[1];
                    boost::trim( frameName );
                    std::string match = matches[2];
                    boost::split( elements, match, boost::is_any_of( "," ) );
                    unsigned j;
                    for( j = 0; j < elements.size( ); ++j ) {
                        boost::trim( elements[j] );
                        std::string elementName = frameName + ":" + elements[j];
                        frames_order[frameCounter++] = elementName;
                        //count how many frames with the same name appear in the
                        //pre-conditions
                        if( frame_elements_count[elementName] > 0 ){
                            frame_elements_count[elementName] = frame_elements_count[elementName] + 1;
                        }else{
                            frame_elements_count[elementName] = 1;
                        }
                    } // for
                } // if
            } // for
              

            
            std::map<unsigned int, std::string>::const_iterator it;
            for( it = frames_order.begin( ); it != frames_order.end( ); ++it ) {
                //std::cout << "FElement: " << it->first << " " << it->second << std::endl;
                logger().debug("FramesToRelexRuleEngine::%s - FrameElement found in order: %d - %s",__FUNCTION__,it->first,it->second.c_str() );
            } // for
            
            //iterate the frame_elements_count to get the preconditions appended with
            //the element count, like: #Color:Color_2, #Entity:Entity_1, and so on
            std::map< std::string, unsigned int>::const_iterator iter;
            for( iter = frame_elements_count.begin(); iter != frame_elements_count.end(); ++iter) {
                std::string precondition = iter->first+"$"+boost::lexical_cast<std::string>(iter->second);
                //std::cout << "Pre-Condition: " << precondition << std::endl;
                logger().debug("FramesToRelexRuleEngine::%s - Pre-Condition found: %s",__FUNCTION__, precondition.c_str());
                preconditions_set.insert( precondition );
            }

            //TODO  check if it is better to replace here than on OutputRelex
            /*
            for( i = 0; i < frameCounter; i++) {
                boost::replace_all( tokens[1], "$"+boost::lexical_cast<std::string>(i+1), "valor$"+frames[i] );//add 1 to id because it starts with 0
            }
            */

            std::vector<std::string> effect;
            boost::split( effect, tokens[1], boost::is_any_of( ";" ) );

            std::vector<std::string>::const_iterator iter_effect;
            for( iter_effect = effect.begin(); iter_effect != effect.end(); ++iter_effect){
                logger().debug("FramesToRelexRuleEngine::%s - Effect found: %s",__FUNCTION__, (*iter_effect).c_str() );
            }

            /*
            std::cout << "preconditions:" << std::endl;
            std::copy( preconditions_set.begin( ), preconditions_set.end( ), std::ostream_iterator<std::string>( std::cout, "\n" ));
            std::cout << "effect:" << std::endl;
            std::copy( effect.begin( ), effect.end( ), std::ostream_iterator<std::string>( std::cout, "\n" ));
            */

            OutputRelex* output_relex = new OutputRelex( effect, frames_order );
            rules.insert( std::pair<std::set<std::string>, OutputRelex*>( preconditions_set, output_relex) );

        }
        rulesFile.close();
    }//if file open 
    else {
        //std::cout << "Unable to open relex2frames file" << std::endl;
        logger().error("FramesToRelexRuleEngine::%s - Unable to open the Frames2Reles rules file named %s.",
                        __FUNCTION__,rulesFileName );
    }

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

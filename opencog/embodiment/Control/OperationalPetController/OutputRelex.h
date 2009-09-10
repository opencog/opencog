/*
 * opencog/embodiment/Control/OperationalPetController/OutputRelex.h
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

#ifndef OUTPUTRELEX_H
#define OUTPUTRELEX_H

#include <opencog/atomspace/AtomSpace.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>
using namespace opencog;

namespace OperationalPetController
{
    class OutputRelex
    {
    private:
        std::vector< std::string> effect;//the output with $1, $2, etc
        std::map<unsigned int, std::string> frames_order;//the order of the frames
    public:
        OutputRelex( ){ }
        OutputRelex( std::vector< std::string > effect,
                                               std::map<unsigned int, std::string> frames_order) :
            effect( effect ), 
            frames_order( frames_order) {}
        /*
         * This method replaces the variables $1, $2 and so on by the
         * corresponding handle values. It iterates the frames_order and find
         * for each one the corresponding handle. When found, it replace the
         * corresponding variable. Then, returns the relex output according to
         * the effect values of the rule
         *
         * XXX It is expected the handles in the order of the expected
         * variables. So, if we have two frames with the same name and elements,
         * the order of the handles must be correct, because there is no way to
         * identify which one must come first here 
         *
         * TODO assert that the number of handles is the same number of
         * variables (frames_order)
         */
        virtual std::string getOutput( const AtomSpace &atomSpace, std::vector< std::pair<std::string, Handle> > handles ) {
            //for each frames_order
                //for each handle
                //if frames_order.name == handle.name
                    //for each effect
                    //if effect.number == frames_order.number
                    //replace effect.number by handle.value
            
            //iterators
            std::map<unsigned int, std::string>::const_iterator iter_frames_order;
            std::vector< std::string>::iterator iter_effect;
            std::vector< std::pair<std::string, Handle> >::iterator iter_handles;
            //for each frame in frames_order 
            for( iter_frames_order = frames_order.begin(); iter_frames_order != frames_order.end(); ++iter_frames_order ){
                //std::cout << "Frame order: " << iter_frames_order->second << " -- " << iter_frames_order->first << std::endl;
                logger().debug("OutputRelex::%s - Frame %s found in order %d.",__FUNCTION__, iter_frames_order->second.c_str(),  iter_frames_order->first);

                //for each handle
                for ( iter_handles = handles.begin(); iter_handles != handles.end(); ++iter_handles ){
                    //std::cout << "Handle: " << iter_handles->first << std::endl;
                    logger().debug("OutputRelex::%s - Handle %s found.",__FUNCTION__, iter_handles->first.c_str() );

                    //if the handle name is the same as the frame order name
                    if ( boost::equals(iter_frames_order->second, iter_handles->first) ){
                       //for each effect, replace the variable of the frame
                       //order by the hande value
                       for ( iter_effect = effect.begin(); iter_effect != effect.end(); ++iter_effect ) {
                            boost::replace_all((*iter_effect) , "$"+boost::lexical_cast<std::string>(iter_frames_order->first+1),  atomSpace.getName(iter_handles->second) );
                            //std::cout <<  "New Effect: " << (*iter_effect) << std::endl;
                            logger().debug("OutputRelex::%s - Effect after replacing the variable %s .",__FUNCTION__, (*iter_effect).c_str() );
                       }//for effect
                       //remove the handle so it will not be used anymore 
                       handles.erase(iter_handles);
                       break;
                    }//if order
                }//for handles
            }//for frames_order

            //create the complete output as all the effects
            std::string output;
            for ( iter_effect = effect.begin(); iter_effect != effect.end(); ++iter_effect ){
                output = output + (*iter_effect) + "\n";
            }
            return output;
        }
    };
};

#endif // OUTPUTRELEX_H

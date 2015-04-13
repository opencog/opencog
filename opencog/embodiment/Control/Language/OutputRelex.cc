/*
 * opencog/embodiment/Control/Language/OutputRelex.cc
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

#include <opencog/nlp/types/atom_types.h>
#include <opencog/spacetime/atom_types.h>

#include <opencog/embodiment/Control/Language/OutputRelex.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

using namespace opencog::oac;

std::string OutputRelex::getOutput( const AtomSpace &atomSpace, const std::vector< std::pair<std::string, Handle> >& handles ) 
{
    //for each frames_order
    //for each handle
    //if frames_order.name == handle.name
    //for each effect
    //if effect.number == frames_order.number
    //replace effect.number by handle.value
    
    //iterators
    std::map<unsigned int, std::string>::const_iterator iter_frames_order;
    std::vector< std::string>::iterator iter_effect;

    std::list< std::pair<std::string, Handle> > usedElements;
    std::copy( handles.begin( ), handles.end( ), std::back_inserter(usedElements) );
    std::list< std::pair<std::string, Handle> >::iterator iter_handles;

    std::vector<std::string> outputEffect = effect;

    //for each frame in frames_order 
    for( iter_frames_order = frames_order.begin(); iter_frames_order != frames_order.end(); ++iter_frames_order ){
        //std::cout << "Frame order: " << iter_frames_order->second << " -- " << iter_frames_order->first << std::endl;
        logger().debug("OutputRelex::%s - Frame %s found in order %d.",__FUNCTION__, iter_frames_order->second.c_str(),  iter_frames_order->first);
        
        //for each handle
        bool elementFound = false;
        for ( iter_handles = usedElements.begin(); !elementFound && iter_handles != usedElements.end(); ++iter_handles ){
            //std::cout << "Handle: " << iter_handles->first << std::endl;
            logger().debug("OutputRelex::%s - Handle %s found.",__FUNCTION__, iter_handles->first.c_str() );
            
            //if the handle name is the same as the frame order name
            if ( boost::equals(iter_frames_order->second, iter_handles->first) ){
                //for each effect, replace the variable of the frame
                //order by the hande value
                for ( iter_effect = outputEffect.begin(); iter_effect != outputEffect.end(); ++iter_effect ) {
                    std::string elementValue = atomSpace.getName(iter_handles->second);
                    
                    if ( atomSpace.getType( iter_handles->second ) == SEME_NODE ) {
                        HandleSeq realObject(2);
                        realObject[0] = Handle::UNDEFINED;
                        realObject[1] = iter_handles->second;
                        Type types[] = {OBJECT_NODE, SEME_NODE};
                        bool lookForSubTypes[] = {true, false};
                        HandleSeq refLinks;
                        atomSpace.getHandlesByOutgoing( back_inserter(refLinks), realObject, &types[0], &lookForSubTypes[0], 2, REFERENCE_LINK, false );
                        
                        if ( refLinks.size( ) > 0 ) {
                            if ( refLinks.size( ) > 1 ) {
                                logger().error( "OutputRelex:%s - It should be linked just one real node to the SemeNode %s but # %d links were found",
                                                __FUNCTION__, elementValue.c_str( ), refLinks.size( ) );
                            } // if
                            elementValue = AtomSpaceUtil::getObjectName( atomSpace, atomSpace.getOutgoing( refLinks[0], 0) );
                        } // if
                    } // if
                    
                    boost::replace_all((*iter_effect) , "$"+boost::lexical_cast<std::string>(iter_frames_order->first+1),  
                                       elementValue );
                    //std::cout <<  "New Effect: " << (*iter_effect) << std::endl;
                    logger().debug("OutputRelex::%s - Effect after replacing the variable %s .",__FUNCTION__, (*iter_effect).c_str() );
                }//for effect
                //remove the handle so it will not be used anymore 
                usedElements.erase(iter_handles);
                elementFound = true;
            }//if order
        }//for handles
    }//for frames_order
    
    //create the complete output as all the effects
    std::string output;
    for ( iter_effect = outputEffect.begin(); iter_effect != outputEffect.end(); ++iter_effect ){
        output = output + (*iter_effect) + "\n";
    }
    
    if (output.empty() ){
        return output;
    }
    
    //NLGen client socket requires a END line to indicates the end of
    //the relex input
    output = output + "END\n";
    return output;
}

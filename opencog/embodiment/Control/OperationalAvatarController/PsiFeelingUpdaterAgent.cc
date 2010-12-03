/*
 * @file opencog/embodiment/Control/OperationalPetController/FeelingUpdaterAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-10-25
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


#include "OPC.h"
#include "FeelingUpdaterAgent.h"

using namespace OperationalPetController;

FeelingUpdaterAgent::~FeelingUpdaterAgent()
{
}

FeelingUpdaterAgent::FeelingUpdaterAgent()
{
    lastTickTime = 0;
    mergedAtomConnection.disconnect();
}

void FeelingUpdaterAgent::connectSignals(AtomSpace& as)
{
    mergedAtomConnection = as.mergeAtomSignal().connect(boost::bind(&FeelingUpdaterAgent::atomMerged, this, _1));
}

void FeelingUpdaterAgent::run(opencog::CogServer *server)
{

    logger().fine(
                 "ImportanceDecayTask - Executing decayShortTermImportance().");
    /* TODO: FeelingUpdaterAgent is responsible for this, not in ActionSelectionAgent!
    // update feelings
    std::map<std::string, std::vector<float> > weightedFeelings;
    { // calculating the mean of all suggested feelings
        std:: string suggestedFeelings = "Cycle: ";
        suggestedFeelings += toString(cycle);
        suggestedFeelings += " - Suggested Feelings {";

        std::set<Feeling>::iterator it;
        for ( it = this->suggestedFeelings.begin( );
                it != this->suggestedFeelings.end( ); ++it ) {

            weightedFeelings[(*it).getName()].push_back((*it).getIntensity());
            suggestedFeelings += ((*it).getName()
                                  + "-"
                                  + toString((*it).getIntensity())
                                  + " ");
        } // for
        suggestedFeelings += "}";
        logger().debug(suggestedFeelings.c_str()) ;

    } // end block

    { // updating feelings values...
        std::map<std::string, float> feelingsUpdatedMap;
        std::map<std::string, std::vector<float> >::iterator it;
        for ( it = weightedFeelings.begin( );
                it != weightedFeelings.end( ); ++it ) {
            unsigned int i;
            float newValue = 0;
            for ( i = 0; i < it->second.size( ); ++i ) {
                newValue += it->second[ i ];
            } // for

            newValue /= it->second.size( );

            float oldValue =
                AtomSpaceUtil::getPredicateValue(*(this->opc->getAtomSpace()),
                                                 it->first, this->petHandle);

            float revisedValue =  reviseValue( oldValue, newValue );

            AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                             it->first,
                                             SimpleTruthValue(revisedValue,
                                                              0.0f),
                                             this->petHandle);

            feelingsUpdatedMap[ it->first ] = revisedValue;
        } // for

        opc->getPAI().sendEmotionalFeelings(this->petName, feelingsUpdatedMap);
    } // end block

    { // decrease and log current feelings
        std::map<std::string, float> feelingsUpdatedMap;
        float feelingsDecreaseFactor =
            config().get_double("RE_FEELINGS_DECREASE_FACTOR");

        foreach(std::string feeling, feelings) {

            float oldValue =
                AtomSpaceUtil::getPredicateValue(*(opc->getAtomSpace()),
                                                 feeling, petHandle);
            float revisedValue =
                limitValue( oldValue - feelingsDecreaseFactor );

            AtomSpaceUtil::setPredicateValue(*(opc->getAtomSpace()), feeling,
                                             SimpleTruthValue(revisedValue,
                                                              0.0f),
                                             petHandle);

            feelingsUpdatedMap[ feeling ] = revisedValue;

            if ( oldValue != revisedValue ) {
                std::string frameInstanceName = this->opc->getPet( ).getPetId( ) + "_" + feeling + "_emotion_directed";
                try {
                    std::map<std::string, Handle> elements;
                    elements["Experiencer"] = opc->getAtomSpace()->addNode( SEME_NODE, this->opc->getPet( ).getPetId( ) );
                    elements["State"] = opc->getAtomSpace()->addNode( CONCEPT_NODE, feeling );
                    AtomSpaceUtil::setPredicateFrameFromHandles( 
                        *(opc->getAtomSpace()), "#Emotion_directed", frameInstanceName,
                            elements, SimpleTruthValue( (revisedValue < 0.5) ? 0.0 : revisedValue, 0.0 ) );        
                        
                } catch ( const opencog::NotFoundException& ex ) {
                    Handle predicateNode = opc->getAtomSpace()->getHandle( PREDICATE_NODE, frameInstanceName );
                    if ( predicateNode != Handle::UNDEFINED ) {
                        AtomSpaceUtil::deleteFrameInstance( *this->opc->getAtomSpace(), predicateNode );
                    } // if
                } // catch
                
            } // if

        } // foreach

        opc->getPAI().sendEmotionalFeelings(this->petName, feelingsUpdatedMap);
    } // end block
    */

}

void FeelingUpdaterAgent::atomMerged(Handle h)
{
    logger().debug("FeelingUpdaterAgent::atomMerged(%lu)", h.value());
    AtomSpace* atomSpace = server().getAtomSpace();
    // Restore the default STI value if it has decayed
    // TODO: Remove this code when the merge of atoms consider the STI values this way as well.
    if (atomSpace->getSTI(h) < AttentionValue::DEFAULTATOMSTI) {
        atomSpace->setSTI(h, AttentionValue::DEFAULTATOMSTI);
    }
}

/*
 * opencog/embodiment/Control/OperationalAvatarController/RuleEngineLearnedTricksHandler.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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
 *
 * Note: This is file is obsolete but might be served as a reference while 
 *       incorporating learning sever to embodiment system. It can safely 
 *       deleted once learning server is successfully incorporated. 
 */

#ifndef RULEENGINELEARNEDTRICKSHANDLER_H
#define RULEENGINELEARNEDTRICKSHANDLER_H

#include "OAC.h"

namespace opencog { namespace oac {

class RuleEngineLearnedTricksHandler
{
public:
    static const std::string LEARNED_TRICK_NODE_NAME;
    static const int REWARD_VALUE;
    static const int PUNISHMENT_VALUE;
    static const int NOT_SELECTED_VALUE;

    RuleEngineLearnedTricksHandler( OAC* oac );

    ~RuleEngineLearnedTricksHandler( );

    void addLearnedSchema( const std::string& schemaName );

    void selectLearnedTrick( std::string& schemaName, std::set<std::string>& arguments ) throw (opencog::NotFoundException);

    void rewardSchema( std::string& schemaName );

    void punishSchema( std::string& schemaName );

    bool hasLearnedTricks( void );

    void update( void );

private:

    void addToSTIValue( Handle link, short value );

    OAC* oac;
    AtomSpace* atomSpace;
    Handle learnedTrickNode;
    std::set<Handle> punishedTricks;
    std::string latestSelectedTrick;
    int numberOfLearnedTricks;
    opencog::RandGen* randGen;
};

} } // namespace opencog::oac

#endif // RULEENGINELEARNEDTRICKSHANDLER_H

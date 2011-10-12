/*
 * opencog/embodiment/Control/OperationalAvatarController/RuleEngineUtil.h
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

#ifndef RULE_ENGINEUTIL_H_
#define RULE_ENGINEUTIL_H_

namespace opencog { namespace oac {

class RuleEngine;

class RuleEngineUtil
{
public:

    RuleEngineUtil( RuleEngine* ruleEngine );
    inline virtual ~RuleEngineUtil( void ) { };

    typedef std::set<Handle> HandleContainer; //not sure a set is really needed

    // return the set of entity handles that are novel
    HandleContainer getNovelEntityHandleSet( void );

    // return the set of object handles that are novel
    HandleContainer getNovelObjectHandleSet( void );

    // return the set of agent handles that are novel
    HandleContainer getNovelAgentHandleSet( void );

    // check if there is some novel object/avatar near pet
    // note this is should be equivalent to:
    // !getNovelEntityHandleSet().empty()
    bool isNovelty( void );

    // check if there is some novel avatar near pet
    bool isAvatarNovelty( void );

    // check if there is some novel object near pet
    bool isObjectNovelty( void );

    // check if there is a requested action
    bool isThereARequestedSchema( void );

private:
    RuleEngine* ruleEngine;

    int cyclesDuringNovelty;
};

} } // namespace opencog::oac

#endif /*NEWRULEENGINEUTIL_H_*/

/*
 * opencog/embodiment/Learning/Filter/EntityRelevanceFilter.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/misc.h>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Node.h>

#include <opencog/nlp/types/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/Temporal.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/spacetime/SpaceServer.h>

#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>
#include <opencog/embodiment/AtomSpaceExtensions/CompareAtomTreeTemplate.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>
#include <opencog/embodiment/WorldWrapper/WorldWrapperUtil.h>

#include "EntityRelevanceFilter.h"

using namespace AvatarCombo;
using namespace opencog::world;
using namespace opencog; /// @todo make it under the namespace opencog

//using namespace opencog::oac;
const std::string emptyString = "";

EntityRelevanceFilter::EntityRelevanceFilter() {}

EntityRelevanceFilter::EntityRelevanceFilter(const SpaceServer::SpaceMap &spaceMap, const std::string& selfID, const std::string& ownerID)
        : _selfID(selfID), _ownerID(ownerID)
{

    // get all handles associated with the given space map
    //spaceMap.findAllEntities(back_inserter(spaceMapHandles));
    spaceMap.findAllEntities(back_inserter(spaceMapIds));
}

EntityRelevanceFilter::~EntityRelevanceFilter()
{
}

const definite_object_set EntityRelevanceFilter::getEntities(Type type) const
{
    // std::string temp;
    definite_object_set entities;

    for (unsigned int i = 0; i < spaceMapIds.size(); i++)
        entities.insert(WorldWrapperUtil::atom_name_to_definite_object(spaceMapIds[i], _selfID, _ownerID));

    logger().debug("EntityRelevanceFilter - Found %d entities (definite_objects).", entities.size());
    return entities;
}

const definite_object_set EntityRelevanceFilter::getEntities(const WorldProvider& wp,
        const std::string& trick_name,
        const std::string& selfID,
        const std::string& ownerID,
        Type type) const
{
    definite_object_set result;
    //get all maps that span over the exemplars of the trick
    Handle h = wp.getAtomSpace().getHandle(CONCEPT_NODE, trick_name);
    OC_ASSERT(h != Handle::UNDEFINED,
                     "EntityRelevanceFilter - There is no CONCEPT_NODE in AtomSpace for trick '%s'.",
                     trick_name.c_str());
    std::list<HandleTemporalPair> retP;
    timeServer().getTimeInfo(std::back_inserter(retP), h,
                                  Temporal(wp.getLatestSimWorldTimestamp()),
                                  TemporalTable::STARTS_BEFORE);
    for (std::list<HandleTemporalPair>::const_iterator ip = retP.begin();
            ip != retP.end(); ++ip) {
        Temporal temp = *(ip->getTemporal());
        HandleSeq resmh;
        timeServer().getMapHandles(back_inserter(resmh), temp.getLowerBound(), temp.getUpperBound());
        for (Handle h : resmh) {
            const SpaceServer::SpaceMap& spacemap = spaceServer().getMap(h);
            //then make union of the object of that map with the result
            EntityRelevanceFilter erf(spacemap, selfID, ownerID);
            definite_object_set ires = erf.getEntities();
            set_union(ires.begin(), ires.end(), result.begin(), result.end(),
                      insert_iterator<definite_object_set>(result,
                                                           result.begin()) );
        }
    }
    return result;
}

const message_set EntityRelevanceFilter::getMessages(const WorldProvider& wp,
        const std::string& trickName,
        const std::string& toID,
        bool exclude_prefix,
        bool strict_within) const
{
    message_set res;
    //get all temporals denoting the start and end of the exemplars of the trick
    Handle h = wp.getAtomSpace().getHandle(CONCEPT_NODE, trickName);
    if (h != Handle::UNDEFINED) {
        std::list<HandleTemporalPair> retP;
        timeServer().getTimeInfo(std::back_inserter(retP), h,
                                      Temporal(wp.getLatestSimWorldTimestamp()),
                                      TemporalTable::STARTS_BEFORE);
        for (std::list<HandleTemporalPair>::const_iterator ip = retP.begin();
                ip != retP.end(); ++ip) {
            Temporal temp = *(ip->getTemporal());
            message_set ms = getMessages(wp, temp, toID, exclude_prefix,
                                         strict_within);
            //perform the union of ms and res into res
            set_union(ms.begin(), ms.end(), res.begin(), res.end(),
                      insert_iterator<message_set>(res, res.begin()));
        }
    }
    return res;
}

const message_set EntityRelevanceFilter::getMessages(const WorldProvider& wp,
        Temporal t,
        const std::string& toID,
        bool exclude_prefix,
        bool strict_within) const
{
    return getMessages(wp.getAtomSpace(), t, toID, exclude_prefix,
                       strict_within);
}

const message_set EntityRelevanceFilter::getMessages(AtomSpace& atomSpace,
        Temporal t,
        const std::string& toID,
        bool exclude_prefix,
        bool strict_within) const
{
    message_set res;
    //check if all atoms of the structure to find are present
    //returns the empty set of messages if one is missing
    Handle action_done_h = atomSpace.getHandle(PREDICATE_NODE,
                           ACTION_DONE_PREDICATE_NAME);
    if (action_done_h == Handle::UNDEFINED)
        return res;
    Handle say_h = atomSpace.getHandle(GROUNDED_SCHEMA_NODE, SAY_SCHEMA_NAME);
    if (say_h == Handle::UNDEFINED)
        return res;
    if (strict_within)
        t = Temporal(t.getLowerBound() + 1, t.getUpperBound() - 1);

    std::list<HandleTemporalPair> htp;
    timeServer().getTimeInfo(back_inserter(htp),
                          Handle::UNDEFINED,
                          t, TemporalTable::STARTS_WITHIN);
    //define template to match
    atom_tree *say_template =  makeVirtualAtom(EVALUATION_LINK,
                               makeVirtualAtom(action_done_h, NULL),
                               makeVirtualAtom(LIST_LINK,
                                               makeVirtualAtom(EXECUTION_LINK,
                                                               makeVirtualAtom(say_h, NULL),
                                                               makeVirtualAtom(LIST_LINK,
                                                                               makeVirtualAtom(OBJECT_NODE, NULL),
                                                                               makeVirtualAtom(SENTENCE_NODE, NULL),
                                                                               NULL
                                                                              ),
                                                               NULL
                                                              ),
                                               NULL
                                              ),
                               NULL
                                              );

    does_fit_template dft(*say_template, &atomSpace, true);
    for (std::list<HandleTemporalPair>::iterator i = htp.begin();
            i != htp.end(); ++i) {
        Handle evalLink_h = i->getHandle();
        //check if evalLink_h match the template
        if (dft(evalLink_h)) {
            //get the sentence and insert it in the res set
            //1 points to ListLink
            //0 points to ExecutionLink
            //1 points to ListLink
            //1 points to SentenceNode
            Handle listLink_h = atomSpace.getOutgoing(evalLink_h, 1);
            Handle executionLink_h = atomSpace.getOutgoing(listLink_h, 0);
            listLink_h = atomSpace.getOutgoing(executionLink_h, 1);
            Handle sentence_h = atomSpace.getOutgoing(listLink_h, 1);

            OC_ASSERT(atomSpace.getType(sentence_h),
                             "sentence_h is not a NODE.");
            std::string message_str = atomSpace.getName(sentence_h);
            if (exclude_prefix) {
                std::string pref = string("to:") + toID + string(": ");
                //check if the message is for toID
                if (message_str.find(pref) == 0) {
                    message m(message_str.substr(pref.size()));
                    res.insert(m);
                }
            } else {
                message m(message_str);
                res.insert(m);
            }
        }
    }
    return res;
}

const agent_to_actions EntityRelevanceFilter::getAgentActions(const WorldProvider& wp,
        const std::string& trick,
        const std::string& selfID,
        const std::string& ownerID,
        const std::set<string>& exclude_set) const
{
    agent_to_actions res;
    //get all temporals denoting the start and end of the exemplars of the trick
    Handle h = wp.getAtomSpace().getHandle(CONCEPT_NODE, trick);
    if (h != Handle::UNDEFINED) {
        std::list<HandleTemporalPair> retP;
        timeServer().getTimeInfo(std::back_inserter(retP), h,
                                      Temporal(wp.getLatestSimWorldTimestamp()),
                                      TemporalTable::STARTS_BEFORE);
        for (std::list<HandleTemporalPair>::const_iterator ip = retP.begin();
                ip != retP.end(); ++ip) {
            Temporal temp = *(ip->getTemporal());
            agent_to_actions atas = getAgentActions(wp.getAtomSpace(),
                                                    temp, selfID, ownerID, exclude_set);
            //insert all elements of atas in res
            for (agent_to_actions_const_it atas_it = atas.begin();
                    atas_it != atas.end(); ++atas_it) {
                agent_to_actions_it res_it = res.find(atas_it->first);
                if (res_it == res.end())
                    res.insert(*atas_it);
                else {
                    definite_object_vec_set& res_ados = res_it->second;
                    const definite_object_vec_set& ados = atas_it->second;
                    //perform the union of ados and res_ados into res_ados
                    set_union(ados.begin(), ados.end(), res_ados.begin(), res_ados.end(),
                              insert_iterator<definite_object_vec_set>(res_ados, res_ados.begin()));
                }
            }
        }
    }
    return res;
}

const agent_to_actions EntityRelevanceFilter::getAgentActions(const WorldProvider& wp,
        const Temporal& t,
        const std::string& selfID,
        const std::string& ownerID,
        const std::set<string>& exclude_set) const
{
    return getAgentActions(wp.getAtomSpace(), t, selfID, ownerID, exclude_set);
}

const agent_to_actions EntityRelevanceFilter::getAgentActions(AtomSpace& as,
        const Temporal& t,
        const std::string& selfID,
        const std::string& ownerID,
        const std::set<string>& exclude_set) const
{

    //std::cout << "GET AGENT ACTIONS : " << t << std::endl;

    agent_to_actions res;
    //check if all atoms of the structure to find are present
    //returns the empty set of messages if one is missing
    Handle action_done_h = as.getHandle(PREDICATE_NODE,
                                        ACTION_DONE_PREDICATE_NAME);
    if (action_done_h == Handle::UNDEFINED)
        return res;
    //if(strict_within)
    //  t = Temporal(t.getLowerBound()+1, t.getUpperBound()-1);

    std::list<HandleTemporalPair> htp;
    timeServer().getTimeInfo(back_inserter(htp),
                   Handle::UNDEFINED,
                   t, TemporalTable::ENDS_WITHIN);
    //define template to match
    atom_tree *no_arg_actionDone =
        makeVirtualAtom(EVALUATION_LINK,
                        makeVirtualAtom(action_done_h, NULL),
                        makeVirtualAtom(LIST_LINK,
                                        makeVirtualAtom(OBJECT_NODE, NULL),
                                        NULL
                                       ),
                        NULL
                       );

    does_fit_template dft(*no_arg_actionDone, &as, false);
    for (std::list<HandleTemporalPair>::const_iterator i = htp.begin();
            i != htp.end(); ++i) {
        Handle evalLink_h = i->getHandle();
        //check if evalLink_h match the template
        if (dft(evalLink_h)) {
            //get listLink
            Handle listLink_h = as.getOutgoing(evalLink_h, 1);
            //get agent
            Handle agent_h = as.getOutgoing(listLink_h, 0);
            std::string agent_id = as.getName(agent_h);
            //get action
            Handle action_h = as.getOutgoing(listLink_h, 1);
            definite_object_vec dov(1, get_action_definite_object(as.getName(action_h)));
            //get action arguments
            for (int a = 2; a < as.getArity(listLink_h); a++) {
                Handle h = as.getOutgoing(listLink_h, a);
                dov.push_back(WorldWrapperUtil::atom_name_to_definite_object(as.getName(h), selfID, ownerID));
            }

            //std::cout << "AGENT ID : " << agent_id << " ACTION NAME : " << action_name << std::endl;

            //get action arguments

            //check if agent_id is not in the exclude_set
            //and add it in the returning set if so
            if (exclude_set.find(agent_id) == exclude_set.end()) {
                definite_object agent_definite_object =
                    WorldWrapperUtil::atom_name_to_definite_object(agent_id,
                            selfID,
                            ownerID);

                agent_to_actions_it ata_it = res.find(agent_definite_object);
                //if that agent_id is not in the agent_to_actions mapping
                //then create a new entry
                if (ata_it == res.end()) {
                    definite_object_vec_set dovs;
                    dovs.insert(dov);
                    std::pair<definite_object, definite_object_vec_set>
                    p(agent_definite_object, dovs);
                    res.insert(p);
                }
                //otherwise just add the new action_name + arguments in the action set
                else {
                    ata_it->second.insert(dov);
                }
            }
        }
    }
    return res;
}

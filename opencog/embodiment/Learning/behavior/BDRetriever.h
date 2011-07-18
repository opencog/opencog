/*
 * opencog/embodiment/Learning/behavior/BDRetriever.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _BDRETRIEVER_H
#define _BDRETRIEVER_H

#include "WorldProvider.h"
#include "CompositeBehaviorDescription.h"
#include "BehaviorCategory.h"

namespace behavior
{

/**
 * That class contains static methods to retrieve and create a
 * BehaviorDescripton or BehaviorCategory representing the exemplar(s)
 * of a certain trick
 */

class BDRetriever
{
public:

    /**
     * retrieve and fill from the WorldProvider 'wp'
     * the object with the exemplar of the trick 'trick_name'
     * that has occured during the interval temp
     * If such an exemplar does not exists then do nothing.
     * The result is added to 'bd'
     */
    static void retrieveExemplar(CompositeBehaviorDescription& bd,
                                 const WorldProvider& wp,
                                 const std::string& trick_name,
                                 const Temporal& temp);
    /**
     * as previously but uses directly the Handle of a trick concept node
     * instead of its name
     */
    static void retrieveExemplar(CompositeBehaviorDescription& bd,
                                 const WorldProvider& wp,
                                 Handle trickConceptNode,
                                 const Temporal& temp);

    /**
     * retrieve and fill from the WorldProvider 'wp'
     * the object with the last exemplar o the trick 'trick_name'
     * If such an exemplar does not exists then do nothing.
     * to work properly the object must be empty
     * if not the method will simply add all elementary BDs corresponding
     * to the last exemplar of the trick 'name'
     * and the start and end time is added in 'exemplarTemporal'
     * that is because the actual start and end time does not
     * necessarily matches the behavior description
     */
    static void retrieveLastExemplar(CompositeBehaviorDescription& bd,
                                     Temporal& exemplarTemporal,
                                     const WorldProvider& wp,
                                     const std::string& name);


    /**
     * retrieve from the WorldProvider 'wp'
     * and fill the object with the last exemplar
     * belonging to the trick 'trick_name'
     * If such an exemplar does not exists then do nothing.
     * and the start and end time is added in 'exemplarsTemporal'
     * that is because the actual start and end time does not
     * necessarily matches the behavior description
     */
    static void addLastExemplar(BehaviorCategory& bc,
                                std::vector<Temporal>& exemplarsTemporal,
                                const WorldProvider& wp,
                                const std::string& trick_name);

    /**
     * retrieve from the WorldProvider 'wp'
     * and fill the object with all exemplars
     * belonging to the trick 'name'
     * If no such exemplars exist then do nothing.
     */
    static void retrieveAllExemplars(BehaviorCategory& bc,
                                     std::vector<Temporal>& exemplarsTemporal,
                                     const WorldProvider& wp,
                                     const std::string& name);

};
}//~namespace behavior

#endif

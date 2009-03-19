/**
 * BDRetriever.h
 *
 * Author(s):
 *   Nil Geisweiller
 * Creation: Thu Sep 6 2007
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
                                 const std::string trick_name,
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

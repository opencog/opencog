#ifndef _SPATIAL_SPACEMAPUTIL_H
#define _SPATIAL_SPACEMAPUTIL_H

#include <string>
#include <set>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>
#include "Octree3DMapManager.h"
#include "Block3DMapUtil.h"

using namespace std;

namespace opencog
{
    namespace spatial
    {

	/**
	 *    PredicateNames
	 */
	const string MATERIAL_PREDICATE="material";

        enum SPATIAL_RELATION
        {
            LEFT_OF = 0,
            RIGHT_OF,
            ABOVE,
            BELOW,
            BEHIND,
            IN_FRONT_OF,
            BESIDE,
            NEAR,
            FAR_,
            TOUCHING, // touching is only face touching
            BETWEEN,
            INSIDE,
            OUTSIDE,

            TOTAL_RELATIONS
        };

        vector<string> getPredicate(AtomSpace& atomspace, const string& predicateName, const HandleSeq& handles, const unsigned& numberOfPredicateValue);
		
	/**
	 *    spatial computation utility, used in Inquery. 
	 *    For polymorphism design in Inquery, we overload the distanceBetween.
	 */

	/**
	 * Find a free point near a given position, at a given distance
	 * @param position Given position
	 * @param distance Maximum distance from the given position to search the free point
	 * @param startDirection Vector that points to the direction of the first rayTrace
	 * @param toBeStandOn if this is true then agent can stand at that position,which means the point should not be on the sky
	 * 
	 */

	BlockVector getNearFreePointAtDistance(const Octree3DMapManager& spaceMap,
                                               const BlockVector& position, 
                                               int distance, 
                                               const BlockVector& startDirection,
                                               bool toBeStandOn);
	double distanceBetween(const Octree3DMapManager& spaceMap,
                               const Handle& objectA,
                               const Handle& objectB);
	double distanceBetween(const Octree3DMapManager& spaceMap,
                               const BlockVector& posA, 
                               const BlockVector& posB);
	double distanceBetween(const Octree3DMapManager& spaceMap,
                               const Handle& objectA, 
                               const BlockVector& posB);
	bool isTwoPositionsAdjacent(const BlockVector& pos1, 
                                    const BlockVector& pos2);
        AxisAlignedBox getBoundingBox(AtomSpace& atomSpace,
                                      const Octree3DMapManager& spaceMap,
                                      const Handle& entity);

	/**
	 * Finds the list of spatial relationships 
	 * that apply to the three entities.
	 * Currently this can only be BETWEEN, 
	 * which states that A is between B and C
	 *
	 * @param observer The observer entity
	 * @param entityB First reference entity
	 * @param entityC Second reference entity
	 *
	 * @return std::vector<SPATIAL_RELATION> 
	 *         a vector of all spatial relations
	 *         among entityA (this entity), entityB (first reference) 
	 *         and entityC (second reference)
	 *
	 */
        
        set<SPATIAL_RELATION> computeSpatialRelations(AtomSpace& atomSpace,
                                                      const Octree3DMapManager& spaceMap,
                                                      const Handle& entityA,
                                                      const Handle& entityB,
                                                      const Handle& entityC = Handle::UNDEFINED,
                                                      const Handle& observer = Handle::UNDEFINED);
        set<SPATIAL_RELATION> computeSpatialRelations(const AxisAlignedBox& boundingboxA,
                                                      const AxisAlignedBox& boundingboxB,
                                                      const AxisAlignedBox& boundingboxC = AxisAlignedBox::ZERO,
                                                      const Handle& observer = Handle::UNDEFINED);
        
	string spatialRelationToString(SPATIAL_RELATION relation);

        //	Handle getBlockEntity(const Handle& blockHandle);
        //	HandleSeq getComposedBlocks(const Handle& blockEntityHandle);

    }
}

#endif

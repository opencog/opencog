#ifndef _SPATIAL_SPACEMAPUTIL_H
#define _SPATIAL_SPACEMAPUTIL_H

#include <string>
#include <set>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Handle.h>
#include "OpencogOcTree.h"
#include "Block3DMapUtil.h"
#include "EntityRecorder.h"

using namespace std;

namespace opencog
{
    namespace spatial
    {
        const double AccessDistance = 2.0;
        /**
         * Get predicate value of given predicate name and argument handles
         * It will search the following pattern:
         * EvaluationLink
         *     PredicateNode $predicateName
         *     ListLink
         *         ArgumentAtoms (input handles)
         *         PredicateValueAtoms (return strings)
         * @TODO This should be included in the embodiment/AtomSpaceUtil,
         *       The SpaceMap module should include this function in AtomSpaceUtil.
         *       For now because the AtomSpaceUtil including spacetime library
         *       It will create a circular dependence
         *       AtomSpaceUtil -> spacetime -> spaceMap -> AtomSpaceUtil...
         *       So CMake will complain it.
         *       To solve this, we should seperate the AtomSpaceUtil to 2 parts
         *       which one is "pure" atomspace utility,
         *       another is utilities using atomspace and spacetime modules.
         *       Then the SpaceMap can include the "pure" atomspace utility directly.
         * @param atomSpace  Given atomspace containing given handles
         *        It's not const because we need to add temporary atom to query info
         * @param predicateName Predicate name string
         * @param handles The argument atoms
         * @param numberOfPredicateValue The numbers of returned predicate values
         *        ex. we need 3 nodes to represent size predicate (x, y, z)
         * @return vector<string> The predicate values in string
         */

        vector<string> getPredicate(AtomSpace& atomSpace,
                                    const string& predicateName,
                                    const HandleSeq& handles,
                                    const unsigned& numberOfPredicateValue);


        /**
         * Check if given position in given spaceMap is standable
         */

        bool checkStandable(AtomSpace& atomSpace, const OpencogOcTree& spaceMap, const BlockVector& pos);
        bool checkStandableWithProb(AtomSpace& atomSpace, const OpencogOcTree& spaceMap, const BlockVector& pos, float logOddsOccupancy);

	/**
	 * Find a free point near a given position, at a given distance
         * @param atomSpace Given atomSpace
         * @param spaceMap Given spaceMap
	 * @param position Given start position
	 * @param distance Maximum distance from the given position to search the free point
	 * @param startDirection Vector that points to the direction of the first rayTrace
	 * @param toBeStandOn if this is true then agent can stand at that position,
         *        which means the point should not be on the sky
	 */

	BlockVector getNearFreePointAtDistance(AtomSpace& atomSpace,
                                               const OpencogOcTree& spaceMap,
                                               const BlockVector& position,
                                               int distance,
                                               const BlockVector& startDirection,
                                               bool toBeStandOn=true);
	/**
	 * Calculate distance between 2 given block/entity handles or positions
	 * For polymorphism design in Inquery, we overload the distanceBetween.
         * @param spaceMap Given SpaceMap
         * @param objectA/posA Given object(block or entity) handle A / position A
         * @param objectB/posB Given object(block or entity) handle B / position B
         * @return Spatial distance between A and B
         *         return DOUBLE_MAX if pos of object A/B not found in spaceMap
         * @TODO Maybe we should raise exception when object not found. Returning
         *       DOUBLE_MAX may make user confused..
	 */

	double distanceBetween(const OpencogOcTree& spaceMap,
                               const EntityRecorder& entityRecorder,
                               const Handle& objectA,
                               const Handle& objectB);
	double distanceBetween(const OpencogOcTree& spaceMap,
                               const BlockVector& posA,
                               const BlockVector& posB);
	double distanceBetween(const OpencogOcTree& spaceMap,
                               const EntityRecorder& entityRecorder,
                               const Handle& objectA,
                               const BlockVector& posB);

	/**
	 * Check if 2 positions are adjacent.
         * By definition, the "adjacent" means:
         * dx <= 1 && dy<= 1 && dz <= 1
         * dx, dy, dz is the absolute difference of x, y, z coordinates
         * @TODO A special case is that if dx = dy = 0 and dz <= 1
         *       It will return false.
         *       Not sure why it was designed so
         *       (This is used in old embodiment, now we just preserve functionality)
         *       For now just save this "feature. In the future we should remove
         *       this behavior..
         * @param pos1 Given position 1
         * @param pos2 Given position 2
         * @return if pos1 and pos2 is adjacent
	 */

	bool isTwoPositionsAdjacent(const BlockVector& pos1,
                                    const BlockVector& pos2);

	/**
	 * Get bounding box of given entity
         * It will query the size info of entity in atomspace and
         * the location info in spaceMap.
         * Then pass them to create a AxisAlignedBox object representing the entity.
         * @param atomSpace Given atomspace containing given entity handle
         *        It's not const because we need to add temporary atom to query info
         * @param entityRecorder Given spaceMap containing given entity handle
         * @param entity Given entity handle
         * @return A bounding box object
	 */

        AxisAlignedBox getBoundingBox(AtomSpace& atomSpace,
                                      const EntityRecorder& entityRecorder,
                                      const Handle& entity);
        /**
         * spatial relation enumeration for expressing state of relation
         * used in the computeSpatialRelations()
         */

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

            TOTAL_RELATIONS,
        };

	/**
	 * Finds the list of spatial relationships(SPATIAL_RELATION enum)
	 * that apply to the three entities.
	 * @TODO We only have finished TOUCHING, ABOVE, BELOW, NEAR, FAR relation
         *       We should complete all the relationships in the SPATIAL_RELATION enum
         * @TODO The observer param is unused now. If we don't use it in the other
         *       spatial relationship we should remove it.
         * @param atomSpace Given atomspace containing given entity handles
         *        It's not const because we need to add temporary atom to query info
         * @param entityRecorder Given spaceMap containing given entity handles
	 * @param entityA the entity to be judged
	 * @param entityB first reference entity
         * @param entityC second reference entity for ternary spatial relation.
         *        Default argument is Handle::UNDEFINED, which means we will not
         *        do any ternary spatial relationship judgement.
         *        It's unused now because we haven't finished BETWEEN relation judgement.
         * @param observer The observer entity for spatial relation about observer.
         *        Default argument is Handle::UNDEFINED.
         *        It's unused now. For keeping the interface in old code, we preserve it.
	 * @return std::vector<SPATIAL_RELATION>
	 *         a vector of all spatial relations
	 *         among entityA (this entity), entityB (first reference)
	 *         and entityC (second reference)
	 */

        set<SPATIAL_RELATION> computeSpatialRelations(AtomSpace& atomSpace,
                                                      const EntityRecorder& entityRecorder,
                                                      const Handle& entityA,
                                                      const Handle& entityB,
                                                      const Handle& entityC = Handle::UNDEFINED,
                                                      const Handle& observer = Handle::UNDEFINED);
        set<SPATIAL_RELATION> computeSpatialRelations(const AxisAlignedBox& boundingboxA,
                                                      const AxisAlignedBox& boundingboxB,
                                                      const AxisAlignedBox& boundingboxC = AxisAlignedBox::ZERO,
                                                      const Handle& observer = Handle::UNDEFINED);

        /**
         * Helper functions to transform SPATIAL_RELATION enum to string
         * @param relation Enumeartion to record spatial relation state
         * @return String to express spatial relation
         */

	string spatialRelationToString(SPATIAL_RELATION relation);

        /** For old interface
         * For the sake of simplification, we only have 8 kinds of direction:
         * (x = 1, y = 0):    -pai/8 <= a < pai/8
         * (x = 1, y = 1):     pai/8 <= a < pai*3/8
         * (x = 0, y = 1):   pai*3/8 <= a < pai*5/8
         * (x = -1, y = 1):  pai*5/8 <= a < pai*7/8
         * (x = -1, y = 0):  pai*7/8 <= a < -pai*7/8
         * (x = -1, y = -1):-pai*7/8 <= a < -pai*5/8
         * (x = 0, y = -1): -pai*5/8 <= a < -pai*3/8
         * (x = 1, y = -1): -pai*5/8 <= a < -pai/8
         * And we assume in the AtomSpace we express rotation of entity as
         * EvaluationLink
         *   PredicateNode "$rotationPredicateName"
         *   ListLink
         *     ObjectNode entity
         *     NumberNode pitch
         *     NumberNode roll
         *     NumberNode yaw
         */
        BlockVector getEntityDirection(AtomSpace& atomSpace, const EntityRecorder& entityRecorder, Handle entity, const string& rotationPredicateName, int yawNodePosition);
    }
}

#endif

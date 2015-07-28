#include <string>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>

using namespace std;

namespace opencog
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


	/**
	 *    getter functions
	 */
	string getPredicate(AtomSpace& atomSpace,
						const string& predicateName,const Handle& blockHandle);

//	Handle getBlockEntity(const Handle& blockHandle);
//	HandleSeq getComposedBlocks(const Handle& blockEntityHandle);
	
	/**
	 *    spatial computation utility
	 */
/*	
	double distanceBetween(const Entity3D* entityA,
						   const Entity3D* entityB) const;
	double distanceBetween(const BlockVector& posA, 
						   const BlockVector& posB) const;
	double distanceBetween(const string& objectNameA, 
						   const string& objectNameB) const;
	double distanceBetween(const string& objectName, 
						   const BlockVector& pos) const;
	bool isTwoPositionsAdjacent(const BlockVector& pos1, 
								const BlockVector& pos2) const;
*/	
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
/*
	std::set<SPATIAL_RELATION> computeSpatialRelations(
		const Entity3D* entityA,
		const Entity3D* entityB,
		const Entity3D* entityC = 0,
		const Entity3D* observer = 0) const;
	std::set<SPATIAL_RELATION> computeSpatialRelations( 
		const AxisAlignedBox& boundingboxA,
		const AxisAlignedBox& boundingboxB,
		const AxisAlignedBox& boundingboxC = AxisAlignedBox::ZERO,
		const Entity3D* observer = 0 ) const;
	static string spatialRelationToString(SPATIAL_RELATION relation);
*/
}

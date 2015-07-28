#include <iterator>
#include <string>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Logger.h>
#include "SpaceMapUtil.h"

using namespace std;

namespace opencog
{
	string getPredicate(AtomSpace& atomspace,
						const string& predicateName,const Handle& blockHandle)
	{
		Handle predicateNode = atomspace.get_handle(PREDICATE_NODE,predicateName);
		if(predicateNode==Handle::UNDEFINED || 
		   blockHandle==Handle::UNDEFINED){ return "";}
		//try getlink
		Handle hVariableNode = atomspace.add_node(VARIABLE_NODE, "$pred_val");
		HandleSeq predicateListLinkOutgoings;
		predicateListLinkOutgoings.push_back(blockHandle);
		predicateListLinkOutgoings.push_back(hVariableNode);
		Handle predicateListLink = atomspace.add_link(LIST_LINK, predicateListLinkOutgoings);
		HandleSeq evalLinkOutgoings;
		evalLinkOutgoings.push_back(predicateNode);
		evalLinkOutgoings.push_back(predicateListLink);
		Handle hEvalLink = atomspace.add_link(EVALUATION_LINK, evalLinkOutgoings);
		HandleSeq getLinkOutgoing;
		getLinkOutgoing.push_back(hEvalLink);
		Handle getLink=atomspace.add_link(GET_LINK,getLinkOutgoing);
		Handle result=satisfying_set(&atomspace,getLink);
		logger().error("the get link result %s",atomspace.atom_as_string(result).c_str());
		HandleSeq resultSet = (LinkCast(result)->getOutgoingSet());
		if(resultSet.empty()){return "";}
		for(auto outgoing: resultSet)
		{
			logger().error("outgoing %s",atomspace.atom_as_string(outgoing).c_str());
		}
		return atomspace.get_name(resultSet[0]);
	}
}

/* 
   The link structure of ComposedOfLink:
   
   ComposedOfLink
       BlockEntityNode "entityA"
       ListLink
           StructureNode "blockB"
           StructureNode "blockC"
           ...
   In the foloowing block entity query we assume one block only belongs to one block entity. But maybe we will have block belongs to multiple block entity.
===>Yes, there should only be one block entity at one time; But in atomspace there may be multiple blockentitynode have the same block, though they appear in different time..
*/

/*
Handle getBlockEntity(const Handle& blockHandle, const AtomSpace& atomspace)
{
	HandleSeq listLinks;
	blockHandle->getIncomingSetByType(back_inserter(listLinks), LIST_LINK, false);
	HandleSeq blockEntities;
	for(auto handle: Listlinks)
	{
		LinkPtr listlink=LinkCast(handle);
		HandleSeq composedOfLink;
		listlink->getIncomingSetByType(back_inserter(composedOfLink),COMPOSED_OF_LINK,false);
		if(!composedOfLink.empty())
		{
			blockEntities.push_back(LinkCast(composedOfLink[0])->getOutgoingAtom(0));
		}
	}
	if(blockEntities.empty()){ return Handle::UNDEFINED;}
	else
	{
		
	}

}

HandleSeq getComposedBlocks(const Handle& blockEntityHandle)
{
	HandleSeq composedOfLinks;
	blockEntityHandle->getIncomingSetByType(back_inserter(composedOfLinks), COMPOSED_OF_LINK, false);
	if(composedOfLinks.empty())
	{
		logger().error("getComposedBlock::the block entity handle %u's composedOfLink not found!",blockEntityHandle);
		return HandleSeq();
	}
	//TODO: for now the get_ougoing() in AtomSpace.h is deprecated. But we'd better to have a function to wrap the opearation..
	LinkPtr lll = LinkCast(composedOfLinks[0]);
	Handle listLink=lll->getOutgoingAtom(1);
	return listLink->getOutgoingSet();
}


double Octree3DMapManager::distanceBetween(const Entity3D* entityA,const Entity3D* entityB) const
{
    return (entityA->getPosition() - entityB->getPosition());
}

double Octree3DMapManager::distanceBetween(const BlockVector& posA, const BlockVector& posB) const
{
    return (posA - posB);
}

double Octree3DMapManager::distanceBetween(const string& objectNameA,const string& objectNameB) const
{
    const Entity3D* entityA = getEntity(objectNameA);
    const Entity3D* entityB = getEntity(objectNameB);
    if (entityA && entityB)
        return distanceBetween(entityA,entityB);
    else
        return DOUBLE_MAX;
}

double Octree3DMapManager::distanceBetween(const string& objectName, const BlockVector& pos) const
{
    const Entity3D* entity = getEntity(objectName);
    if (! entity)
        return DOUBLE_MAX;

    return (entity->getPosition() - pos);

}

bool Octree3DMapManager::isTwoPositionsAdjacent(const BlockVector &pos1, const BlockVector &pos2)
{
    int d_x = pos1.x - pos2.x;
    int d_y = pos1.y - pos2.y;
    int d_z = pos1.z - pos2.z;
    if (( d_x >=-1) && (d_x <= 1) &&
        ( d_y >=-1) && (d_y <= 1) &&
        ( d_z >=-1) && (d_z <= 1))
    {
		if ((d_x == 0) && (d_y == 0))
			return false; // the position just above or under is considered not accessable
		else
			return true;
    }

    return false;
}

// todo: to be completed

std::set<SPATIAL_RELATION> Octree3DMapManager::computeSpatialRelations( const AxisAlignedBox& boundingboxA,
                                                                        const AxisAlignedBox& boundingboxB,
                                                                        const AxisAlignedBox& boundingboxC,
                                                                        const Entity3D* observer ) const
{
	std::set<SPATIAL_RELATION> spatialRelations;

    if (boundingboxC != AxisAlignedBox::ZERO)
    {
        // todo: compute if A is between B and C

        return spatialRelations;
    }

    if (observer == 0 )
        observer = selfAgentEntity;

    if (boundingboxA.isFaceTouching(boundingboxB))
    {
        spatialRelations.insert(TOUCHING);
    }

    if (boundingboxA.nearLeftBottomConer.z >= boundingboxB.nearLeftBottomConer.z + boundingboxB.size_z)
		spatialRelations.insert(ABOVE);

    if (boundingboxB.nearLeftBottomConer.z >= boundingboxA.nearLeftBottomConer.z + boundingboxA.size_z)
		spatialRelations.insert(BELOW);


    // if A is near/far to B
    double dis = boundingboxB.getCenterPoint() - boundingboxA.getCenterPoint();
    double AR = boundingboxA.getRadius();
    double BR = boundingboxB.getRadius();
    double nearDis = (AR + BR)*2.0;
    if (dis <= nearDis )
		spatialRelations.insert(NEAR);
    else if (dis > nearDis*10.0 )
		spatialRelations.insert(FAR_);

    return spatialRelations;
}


std::set<SPATIAL_RELATION> Octree3DMapManager::computeSpatialRelations( const Entity3D* entityA,
																		const Entity3D* entityB,
																		const Entity3D* entityC,
																		const Entity3D* observer ) const
{
	if (entityC)
	{
		return computeSpatialRelations(entityA->getBoundingBox(),entityB->getBoundingBox(),entityC->getBoundingBox(),observer);
	}
	else
	{
        return computeSpatialRelations(entityA->getBoundingBox(),entityB->getBoundingBox(),AxisAlignedBox::ZERO,observer);
	}
}


std::string Octree3DMapManager::spatialRelationToString( SPATIAL_RELATION relation ) 
{
	switch( relation ) {
	case LEFT_OF: return "left_of";
	case RIGHT_OF: return "right_of";
	case ABOVE: return "above";
	case BELOW: return "below";
	case BEHIND: return "behind";
	case IN_FRONT_OF: return "in_front_of";
	case BESIDE: return "beside";
	case NEAR: return "near";
	case FAR_: return "far";
	case TOUCHING: return "touching";
	case BETWEEN: return "between";
	case INSIDE: return "inside";
	case OUTSIDE: return "outside";
	default:
	case TOTAL_RELATIONS:
		return " invalid relation ";
	}
}
*/



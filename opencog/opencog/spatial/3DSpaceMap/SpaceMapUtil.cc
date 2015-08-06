#include <iterator>
#include <string>
#include <set>

//#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/spacetime/atom_types.h>
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
    namespace spatial
    {
        vector<string> getPredicate(AtomSpace& atomspace, const string& predicateName, const HandleSeq& handles, const unsigned& numberOfPredicateValue)
        {
            vector<string> result;
            Handle predicateNode = atomspace.get_handle(PREDICATE_NODE,predicateName);
            if(predicateNode == Handle::UNDEFINED){
                return result;
            }

            HandleSeq predicateListLinkOutgoings;
            for(auto handle : handles) {
                predicateListLinkOutgoings.push_back(handle);
            }
            for(unsigned i=0; i != numberOfPredicateValue ; i++) {
                string hVariableNodeName = "$pred_val" + std::to_string(i);
                Handle hVariableNode = atomspace.add_node(VARIABLE_NODE, hVariableNodeName);
                predicateListLinkOutgoings.push_back(hVariableNode);
            }
            Handle predicateListLink = atomspace.add_link(LIST_LINK, predicateListLinkOutgoings);
            HandleSeq evalLinkOutgoings;
            evalLinkOutgoings.push_back(predicateNode);
            evalLinkOutgoings.push_back(predicateListLink);
            Handle hEvalLink = atomspace.add_link(EVALUATION_LINK, evalLinkOutgoings);
            HandleSeq getLinkOutgoing;
            getLinkOutgoing.push_back(hEvalLink);
            Handle getLink = atomspace.add_link(GET_LINK,getLinkOutgoing);
            Handle resultSetLink = satisfying_set(&atomspace,getLink);
            logger().error("the get link result %s",atomspace.atom_as_string(resultSetLink).c_str());
            if(numberOfPredicateValue == 1){
                HandleSeq resultNodeSet = (LinkCast(resultSetLink)->getOutgoingSet());
                if(resultNodeSet.empty()) {
                    return result;
                }
                result.push_back(NodeCast(resultNodeSet[0])->getName());
            } else {
                //more than one predicate value, ex.size
                HandleSeq resultListLinkSet = (LinkCast(resultSetLink)->getOutgoingSet());
                if(resultListLinkSet.empty()) {
                    return result;
                }
                HandleSeq resultListLinkOutgoings = LinkCast(resultListLinkSet[0])->getOutgoingSet();
                for(auto predicateValueNode : resultListLinkOutgoings) {
                    result.push_back(NodeCast(predicateValueNode)->getName());
                }
            }
            return result;
        }
        
        BlockVector getNearFreePointAtDistance( const Octree3DMapManager& spaceMap, const BlockVector& position, int distance, const BlockVector& startDirection , bool toBeStandOn) 
        {
            int ztimes = 0;
            int z ;
            Handle block;
            
            while (ztimes <3) {
                // we'll first search for the grids of the same high, so begin with z = 0,
                // then search for the lower grids (z = -1), then the higher grids (z = 1)
                if (ztimes == 0) {
                    z = 0;
                } else if(ztimes == 1) {
                    z = -1;
                } else { 
                    z = 1;
                }                
                ztimes++;
		
                // we first search at the startdirection, if cannot find a proper position then go on with the complete search
                BlockVector curpos(position.x + startDirection.x, position.y + startDirection.y, position.z + z);
                if (toBeStandOn) {
                    if(spaceMap.checkStandable(curpos)) {
                        return curpos;
                    }
                }
                else if(spaceMap.getBlock(curpos) == Handle::UNDEFINED) {
                    return curpos;
                }
                		
                for (int dis = 1; dis <= distance; dis++) {

                    for (int x = 0; x <= dis; x++) {
                        BlockVector curpos(position.x + x, position.y + dis,position.z + z);
                        if(toBeStandOn) {
                            if(spaceMap.checkStandable(curpos)){
                                return curpos;
                            }
                        } else if(spaceMap.getBlock(curpos) == Handle::UNDEFINED) { 
                                return curpos;
                            }
                    }
			
                    for (int y = 0; y <= dis; y++) {
                        BlockVector curpos(position.x + dis, position.y + y,position.z + z);
                        if (toBeStandOn) {
                            if(spaceMap.checkStandable(curpos)) {
                                return curpos;
                            }
                        } else if(spaceMap.getBlock(curpos) == Handle::UNDEFINED) {
                                return curpos;
                        }
                    }
                }
            }            
            return BlockVector::ZERO;
        }
		
		
        double distanceBetween(const Octree3DMapManager& spaceMap,const Handle& objectA,const Handle& objectB)
        {
            BlockVector posA,posB;
			
            Type typeA=objectA->getType();
            if(typeA==ENTITY_NODE){ posA=spaceMap.getLastAppearedLocation(objectA);}
            else if(typeA==STRUCTURE_NODE){ posA=spaceMap.getBlockLocation(objectA);}
			
            Type typeB=objectB->getType();
            if(typeA==ENTITY_NODE){ posB=spaceMap.getLastAppearedLocation(objectB);}
            else if(typeB==STRUCTURE_NODE){ posB=spaceMap.getBlockLocation(objectB);}
			
            if(posA==BlockVector::ZERO || posB==BlockVector::ZERO)
            { return DOUBLE_MAX;}
            else 
            { return posA-posB;}
        }
		
        double distanceBetween(const Octree3DMapManager& spaceMap, const BlockVector& posA, const BlockVector& posB)
        {
            return (posA - posB);
        }
		
        double distanceBetween(const Octree3DMapManager& spaceMap, const Handle& objectA, const BlockVector& posB) 
        {
            BlockVector posA;
            Type typeA=objectA->getType();
            if(typeA==STRUCTURE_NODE){ posA=spaceMap.getBlockLocation(objectA);}
            else if(typeA==ENTITY_NODE){ posA=spaceMap.getLastAppearedLocation(objectA);}
            
            if(posA==BlockVector::ZERO){ return DOUBLE_MAX;}
            return (posA - posB);

        }

        bool isTwoPositionsAdjacent(const BlockVector &pos1, const BlockVector &pos2)
        {
            int d_x = pos1.x - pos2.x;
            int d_y = pos1.y - pos2.y;
            int d_z = pos1.z - pos2.z;
            if (( d_x >=-1) && (d_x <= 1) &&
                ( d_y >=-1) && (d_y <= 1) &&
                ( d_z >=-1) && (d_z <= 1)) {
                if ((d_x == 0) && (d_y == 0)){
                    // the position just above or under is considered not accessable
                    return false;
                } else {
                    return true;
                }
            }
			
            return false;
        }

        
        AxisAlignedBox getBoundingBox(AtomSpace& atomSpace,
                                      const Octree3DMapManager& spaceMap,
                                      const Handle& entity)
        {
            BlockVector nearLeftPos = spaceMap.getLastAppearedLocation(entity);
            vector<string> sizeStrings = getPredicate(atomSpace, "size", 
                                                      HandleSeq({entity}), 3);
            double length = std::stof(sizeStrings[0]);
            double width = std::stof(sizeStrings[1]);
            double height = std::stof(sizeStrings[2]);
            
            return AxisAlignedBox(nearLeftPos, length, width, height);
        }
        

        
        std::set<SPATIAL_RELATION> computeSpatialRelations(AtomSpace& atomSpace,
                                                           const Octree3DMapManager& spaceMap,
                                                           const Handle& entityA,
                                                           const Handle& entityB,
                                                           const Handle& entityC,
                                                           const Handle& observer)
        {
            AxisAlignedBox boxA = getBoundingBox(atomSpace, spaceMap, entityA);
            AxisAlignedBox boxB = getBoundingBox(atomSpace, spaceMap, entityB);
            AxisAlignedBox boxC = (entityC == Handle::UNDEFINED) 
                ? AxisAlignedBox::ZERO
                : getBoundingBox(atomSpace, spaceMap, entityC);
            return computeSpatialRelations(boxA, boxB, boxC, observer);
        }

        
        std::set<SPATIAL_RELATION> computeSpatialRelations(const AxisAlignedBox& boundingboxA,
                                                           const AxisAlignedBox& boundingboxB,
                                                           const AxisAlignedBox& boundingboxC,
                                                           const Handle& observer)
        {
            std::set<SPATIAL_RELATION> spatialRelations;
            
            if (boundingboxC != AxisAlignedBox::ZERO) {
                // todo: compute if A is between B and C
                return spatialRelations;
            }
            
            if (boundingboxA.isFaceTouching(boundingboxB)) {
                spatialRelations.insert(TOUCHING);
            }
            
            if (boundingboxA.nearLeftBottomConer.z >= boundingboxB.nearLeftBottomConer.z + boundingboxB.size_z) {
                spatialRelations.insert(ABOVE);
            }
            
            if (boundingboxB.nearLeftBottomConer.z >= boundingboxA.nearLeftBottomConer.z + boundingboxA.size_z) {
                spatialRelations.insert(BELOW);
            }
            // if A is near/far to B
            double dis = boundingboxB.getCenterPoint() - boundingboxA.getCenterPoint();
            double AR = boundingboxA.getRadius();
            double BR = boundingboxB.getRadius();
            double nearDis = (AR + BR) * 2.0;
            if (dis <= nearDis) {
                spatialRelations.insert(NEAR);
            } else if (dis > nearDis*10.0 ) {
                spatialRelations.insert(FAR_);
            }
            return spatialRelations;
        }



        std::string spatialRelationToString( SPATIAL_RELATION relation ) 
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


          // todo: to be completed

          */
    }
}


#include <limits.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Logger.h>
#include <opencog/spatial/math/Quaternion.h>
#include <opencog/spatial/math/Vector3.h>
#include "SpaceMapUtil.h"

// used for distanceBetween()
#define DOUBLE_MAX numeric_limits<double>::max()

using namespace std;

namespace opencog
{
    namespace spatial
    {
        vector<string> getPredicate(AtomSpace& atomSpace,
                                    const string& predicateName,
                                    const HandleSeq& handles,
                                    const unsigned& numberOfPredicateValue)
        {
            vector<string> result;
            Handle predicateNode = atomSpace.get_handle(PREDICATE_NODE,predicateName);
            if (predicateNode == Handle::UNDEFINED) {
                return result;
            }

            HandleSeq predicateListLinkOutgoings;
            for (auto handle : handles) {
                predicateListLinkOutgoings.push_back(handle);
            }
            for (unsigned i = 0; i != numberOfPredicateValue ; i++) {
                string hVariableNodeName = "$pred_val" + std::to_string(i);
                Handle hVariableNode = atomSpace.add_node(VARIABLE_NODE, hVariableNodeName);
                predicateListLinkOutgoings.push_back(hVariableNode);
            }
            Handle predicateListLink = atomSpace.add_link(LIST_LINK, predicateListLinkOutgoings);
            HandleSeq evalLinkOutgoings;
            evalLinkOutgoings.push_back(predicateNode);
            evalLinkOutgoings.push_back(predicateListLink);
            Handle hEvalLink = atomSpace.add_link(EVALUATION_LINK, evalLinkOutgoings);
            HandleSeq getLinkOutgoing;
            getLinkOutgoing.push_back(hEvalLink);
            Handle getLink = atomSpace.add_link(GET_LINK, getLinkOutgoing);
            Handle resultSetLink = satisfying_set(&atomSpace, getLink);

            if (numberOfPredicateValue == 1) {
                HandleSeq resultNodeSet = (LinkCast(resultSetLink)->getOutgoingSet());
                if (resultNodeSet.empty()) {
                    return result;
                }
                result.push_back(NodeCast(resultNodeSet[0])->getName());
            } else {
                //more than one predicate value, ex.size
                HandleSeq resultListLinkSet = (LinkCast(resultSetLink)->getOutgoingSet());
                if (resultListLinkSet.empty()) {
                    return result;
                }
                HandleSeq resultListLinkOutgoings = LinkCast(resultListLinkSet[0])->getOutgoingSet();
                for (auto predicateValueNode : resultListLinkOutgoings) {
                    result.push_back(NodeCast(predicateValueNode)->getName());
                }
            }
            return result;
        }


        bool checkStandable(AtomSpace& atomSpace, const AtomOcTree<Handle>& spaceMap, const BlockVector& pos)
        {
            return checkStandableWithProb(atomSpace, spaceMap, pos, spaceMap.getOccupancyThresLog());
        }

        bool checkStandableWithProb(AtomSpace& atomSpace, const AtomOcTree<Handle>& spaceMap, const BlockVector& pos, float logOddsOccupancy)
        {
            if (spaceMap.checkIsOutOfRange(pos)) {
                logger().error("checkstandable: You want to check a pos which outside the limit of the map: at x = %f, y = %f, z= %f ! /n",
                       pos.x,pos.y,pos.z);
                return false;
            }
            // check if there is any non-block obstacle in this pos
            Handle blockHandle = spaceMap.getBlock(pos,logOddsOccupancy);
            if (blockHandle != Handle::UNDEFINED) {
                return false;
            }
            // because the agent has a height,
            // we should check if there are enough room above this pos for this agent to stand on
            float agentHeight = spaceMap.getAgentHeight();
            if (agentHeight > 1) {
                for (int height = 1; height < agentHeight; height ++) {
                    BlockVector blockAbove(pos.x, pos.y, pos.z + height);
                    if (spaceMap.getBlock(blockAbove, logOddsOccupancy) != Handle::UNDEFINED) {
                        return false;
                    }
                }
            }

            BlockVector under(pos.x, pos.y, pos.z - 1);
            Handle underBlock = spaceMap.getBlock(under, logOddsOccupancy);
            if (underBlock != Handle::UNDEFINED) {
                //TODO:Judge if this block is standable
                //if agent can't stand on it (ex.water/lava)return false
                HandleSeq blocks;
                blocks.push_back(underBlock);
                vector<string> materialPredicates = getPredicate(atomSpace, "material", blocks, 1);
                if (materialPredicates.empty()) {
                    logger().error("checkStandable - underBlock is not undefined but no material predicate!");
                    return false;
                }
                string materialOfUnderBlock = materialPredicates[0];

                if (materialOfUnderBlock == "water") {
                    return false;
                } else {
                    return true;
                }
            }
            return false;
        }



        BlockVector getNearFreePointAtDistance(AtomSpace& atomSpace,
                                               const AtomOcTree<Handle>& spaceMap,
                                               const BlockVector& position,
                                               int distance,
                                               const BlockVector& startDirection,
                                               bool toBeStandOn)
        {
            logger().error("getNearFreePoint: dest %f, %f, %f, dist %d", position.x, position.y, position.z, distance);
            int ztimes = 0;
            int z = 0;
            while (ztimes < 3) {
                // we'll first search for the grids of the same high, so begin with z = 0,
                // then search for the lower grids (z = -1), then the higher grids (z = 1)
                if (ztimes == 0) {
                    z = 0;
                } else if (ztimes == 1) {
                    z = -1;
                } else {
                    z = 1;
                }
                ztimes++;

                // we first search at the start direction,
                // if cannot find a proper position then go on with the complete search
                BlockVector curpos(position.x + startDirection.x, position.y + startDirection.y, position.z + z);
                if (toBeStandOn) {
                    if (checkStandable(atomSpace, spaceMap, curpos)) {
                        return curpos;
                    }
                }
                else {
                    if (spaceMap.getBlock(curpos) == Handle::UNDEFINED) {
                        return curpos;
                    }
                }

                for (int dis = 1; dis <= distance; dis++) {
                    for (int x = -dis; x <= dis; x++) {
                        for(int y = -dis; y <= dis; y++) {
                            BlockVector curpos(position.x + x, position.y + y, position.z + z);
                            if (curpos == position) {
                                continue;
                            }
                            if (toBeStandOn) {
                                if (checkStandable(atomSpace, spaceMap, curpos)) {
                                    return curpos;
                                }
                            } else {
                                if(spaceMap.getBlock(curpos) == Handle::UNDEFINED) {
                                    return curpos;
                                }
                            }
                        }
                    }
                }
            }
            return BlockVector::ZERO;
        }

        double distanceBetween(const AtomOcTree<Handle>& spaceMap,
                               const EntityRecorder& entityRecorder,
                               const Handle& objectA,
                               const Handle& objectB)
        {
            BlockVector posA, posB;

            Type typeA = objectA->getType();
            if (typeA == ENTITY_NODE) {
                posA = entityRecorder.getLastAppearedLocation(objectA);
            } else if (typeA==STRUCTURE_NODE) {
                posA = spaceMap.getBlockLocation(objectA);
            }
            Type typeB = objectB->getType();
            if (typeB == ENTITY_NODE) {
                posB = entityRecorder.getLastAppearedLocation(objectB);
            } else if (typeB == STRUCTURE_NODE) {
                posB = spaceMap.getBlockLocation(objectB);
            }

            if (posA == BlockVector::ZERO || posB == BlockVector::ZERO) {
                return DOUBLE_MAX;
            } else {
                return posA - posB;
            }
        }

        double distanceBetween(const AtomOcTree<Handle>& spaceMap,
                               const BlockVector& posA,
                               const BlockVector& posB)
        {
            return (posA - posB);
        }

        double distanceBetween(const AtomOcTree<Handle>& spaceMap,
                               const EntityRecorder& entityRecorder,
                               const Handle& objectA,
                               const BlockVector& posB)
        {
            BlockVector posA;
            Type typeA = objectA->getType();
            if (typeA == STRUCTURE_NODE) {
                posA = spaceMap.getBlockLocation(objectA);
            } else if (typeA==ENTITY_NODE) {
                posA = entityRecorder.getLastAppearedLocation(objectA);
            }

            if (posA == BlockVector::ZERO) {
                return DOUBLE_MAX;
            }

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
                if ((d_x == 0) && (d_y == 0)) {
                    // the position just above or under is considered not accessable
                    return false;
                } else {
                    return true;
                }
            }

            return false;
        }


        AxisAlignedBox getBoundingBox(AtomSpace& atomSpace,
                                      const EntityRecorder& entityRecorder,
                                      const Handle& entity)
        {
            BlockVector nearLeftPos = entityRecorder.getLastAppearedLocation(entity);
            vector<string> sizeStrings = getPredicate(atomSpace, "size",
                                                      HandleSeq({entity}), 3);
            double length = std::stof(sizeStrings[0]);
            double width = std::stof(sizeStrings[1]);
            double height = std::stof(sizeStrings[2]);

            return AxisAlignedBox(nearLeftPos, length, width, height);
        }

        set<SPATIAL_RELATION> computeSpatialRelations(AtomSpace& atomSpace,
                                                      const EntityRecorder& entityRecorder,
                                                      const Handle& entityA,
                                                      const Handle& entityB,
                                                      const Handle& entityC,
                                                      const Handle& observer)
        {
            AxisAlignedBox boxA = getBoundingBox(atomSpace, entityRecorder, entityA);
            AxisAlignedBox boxB = getBoundingBox(atomSpace, entityRecorder, entityB);
            AxisAlignedBox boxC = (entityC == Handle::UNDEFINED)
                ? AxisAlignedBox::ZERO
                : getBoundingBox(atomSpace, entityRecorder, entityC);
            return computeSpatialRelations(boxA, boxB, boxC, observer);
        }

        set<SPATIAL_RELATION> computeSpatialRelations(const AxisAlignedBox& boundingboxA,
                                                      const AxisAlignedBox& boundingboxB,
                                                      const AxisAlignedBox& boundingboxC,
                                                      const Handle& observer)
        {
            set<SPATIAL_RELATION> spatialRelations;

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

        string spatialRelationToString(SPATIAL_RELATION relation ) {
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

        BlockVector getEntityDirection(AtomSpace& atomSpace, const EntityRecorder& entityRecorder, Handle entity, const string& rotationPredicateName, int yawNodePosition)
        {
            if (!entityRecorder.containsEntity(entity)) {
                return BlockVector::ZERO;
            } else {
                //get yaw
                std::vector<std::string> rotationStrs = getPredicate(atomSpace, rotationPredicateName, HandleSeq{entity}, 3);
                if (rotationStrs.empty()) {
                    return BlockVector::ZERO;
                }

                double yaw = std::stod(rotationStrs[yawNodePosition]);
                spatial::math::Quaternion orientation = spatial::math::Quaternion( spatial::math::Vector3::Z_UNIT, yaw);
                spatial::math::Vector3 mathVec = orientation.rotate( spatial::math::Vector3::X_UNIT );
                int x = 1, y = 1;
                if (mathVec.x < 0) {
                    x = -1;
                }

                if (mathVec.y < 0) {
                    y = -1;
                }

                // calculate the tan of the angle
                double tan = mathVec.y / mathVec.x;
                if (tan < 0) {
                    tan *= -1.0f;
                }

                // For the sake of simplification, we only have 8 kinds of direction:
                // (x = 1, y = 0):    -pai/8 <= a < pai/8
                // (x = 1, y = 1):     pai/8 <= a < pai*3/8
                // (x = 0, y = 1):   pai*3/8 <= a < pai*5/8
                // (x = -1, y = 1):  pai*5/8 <= a < pai*7/8
                // (x = -1, y = 0):  pai*7/8 <= a < -pai*7/8
                // (x = -1, y = -1):-pai*7/8 <= a < -pai*5/8
                // (x = 0, y = -1): -pai*5/8 <= a < -pai*3/8
                // (x = 1, y = -1): -pai*5/8 <= a < -pai/8

                // tan(pai/8) = 0.41414356f
                // tan(pai*3/8) = 2.41421356f

                if (tan < 0.41414356f) {
                    // x >> y, so |x| -> 1, y -> 0
                    y = 0;
                } else if (tan > 2.41421356f) {
                    // x <<y , so x -> 0, |y| -> 1
                    x = 0;
                }
                // else x approximate to y, so |x| -> 1, |y| -> 1
                return BlockVector(x,y,0);

            }

        }

    }
}

/*
 * opencog/spatial/3DSpaceMap/Pathfinder3D.cc
 *
 * Copyright (C) 2002-2011 OpenCog Foundation
 * All Rights Reserved
 * Author(s): Shujing Ke
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

#include "Pathfinder3D.h"
#include <set>
#include <map>
#include <iterator>

using namespace opencog;
using namespace opencog::spatial;

bool Pathfinder3D::AStar3DPathFinder(Octree3DMapManager *mapManager,  const BlockVector& begin, const BlockVector& target, vector<BlockVector>& path,
                                     BlockVector& nearestPos, BlockVector& bestPos, bool getNearestPos, bool getBestPos)
{
    BlockVector end = target;
    map<BlockVector,double> costMap;
    map<BlockVector,double>::const_iterator itercost;
    int searchTimes = 0;
    float nearestDis = begin - target;
    float bestHeuristic = nearestDis * 1.41421356f;
    nearestPos = begin;
    bestPos = begin;
    bool nostandable = false;

    // check if the begin and target pos standable first
    if ((! mapManager->checkStandable(begin)) || (! mapManager->checkStandable(target)))
    {
        nostandable = true;
        if ((! getNearestPos) && (! getBestPos))
        return false;
    }

    while(true)
    {
        set<BlockVector> searchedList;
        set<BlockVector>::const_iterator iter;
        BlockVector curPos;

        vector<BlockVector> currentPath;

        currentPath.push_back(begin);
        searchedList.insert(begin);
        int pathfindingSteps = 0;

        bool is_found = false;
        searchTimes ++;
        while(currentPath.size() != 0)
        {
            // successfully achieve the target
            if (currentPath.back() == end)
            {
                is_found = true;
                break;
            }

            // Calculate 24 neighbour cost and then pick the lowest one.
            double lowestCost = 999999.99;
            BlockVector lowestCostPos;
            double curCost;

            for (int i = -1; i < 2; i ++)
            {
                for (int j = -1; j < 2; j ++)
                {
                    for (int k = -1; k < 2; k ++)
                    {
                        // Will not calculate the pos just above or below  or equal to the begin position.
                        if ( (i == 0) && (j == 0))
                            continue;

                        curPos.x = currentPath.back().x + i;
                        curPos.y = currentPath.back().y + j;
                        curPos.z = currentPath.back().z + k;

                        iter = searchedList.find(curPos);
                        if (iter != searchedList.end())
                        {
                            // already searched!
                            continue;
                        }

                        // check if standable
                        if (! mapManager->checkStandable(curPos))
                        {
                            searchedList.insert(curPos);
                            continue;
                        }


                        if ( ! checkNeighbourAccessable(mapManager,currentPath.back() , i, j, k))
                            continue;

                        double costFromLastStep = 0.0;
                        if (k == 1) // this pos is higher than last pos, have to cost more to jump up
                            costFromLastStep = 0.2;
                        else if (k == -1) // this pos is lower than last pos, have to cost more to jump down
                            costFromLastStep = 0.1;

                        itercost = costMap.find(curPos);
                        if (itercost != costMap.end())
                        {
                            if ((double)(itercost->second) + costFromLastStep < lowestCost)
                            {
                                lowestCost = (double)(itercost->second) + costFromLastStep;
                                lowestCostPos = curPos;
                            }
                        }

                        // calculate the cost of this pos
                        curCost = calculateCostByDistance(begin, end, curPos,nearestDis,nearestPos,bestHeuristic,bestPos);

                        pathfindingSteps++;
                        costMap.insert(pair<BlockVector, int>(curPos,curCost));
                        if (curCost + costFromLastStep < lowestCost)
                        {
                            lowestCost = curCost +costFromLastStep;
                            lowestCostPos = curPos;
                        }
                    }
                }
            }

            // if the lowestCost still == 99999.99, it shows that all the neighbours have been searched and failed,
            // so we should return to last step
            if (lowestCost > 999999.0)
            {
                currentPath.pop_back();
            }
            else
            {
                // add the lowestcost neighbour pos to the currentPath
                currentPath.push_back(lowestCostPos);
                searchedList.insert(lowestCostPos);

            }

        }

        if (! is_found)
            return false;

        if ((currentPath.size() < 21) || (searchTimes > 4))
        {
            path.insert(path.begin(), currentPath.begin(), currentPath.end());
            if (nostandable)
                return false;
            else
                return true;
        }

        // find the farthest pos in this currrentPath from the end position
        vector<BlockVector>::iterator disIter, farPosIter;
        double farthestDis = 0.0;
        for(disIter = currentPath.begin(); disIter != currentPath.end(); ++ disIter)
        {
           double dis = (end - *disIter)  + (begin - *disIter);
           if ( dis > farthestDis)
           {
               farPosIter = disIter;
               farthestDis = dis;
           }
        }

        path.insert(path.begin(), farPosIter, currentPath.end());

        // The shortest way found!
        if (farPosIter == currentPath.begin())
        {
            if (nostandable)
                return false;
            else
                return true;
        }

        end = *(--farPosIter);

    }

}


double Pathfinder3D::calculateCostByDistance(const BlockVector& begin, const BlockVector& target,  const BlockVector& pos,
                                             float& nearestDis,BlockVector& nearestPos, float& bestHeuristic, BlockVector& bestPos)
{
    float dis = target - pos;
    if (dis < nearestDis)
    {
        nearestDis = dis;
        nearestPos = pos;
    }

    float heuristic = dis*1.41421356f + (begin - pos);
    if (heuristic < bestHeuristic )
    {
        bestHeuristic = heuristic;
        bestPos = pos;
    }

    return dis;
}


// before call this funciton, please make sure the pos want to access is standable first
bool Pathfinder3D::checkNeighbourAccessable(Octree3DMapManager *mapManager, BlockVector& lastPos, int i, int j, int k)
{
    // if want to access the pos 1 unit lower than last pos
    if (k == -1)
    {
        for (int h = 1; h <= mapManager->getAgentHeight(); h++)
        {
            if (i != 0)
            {
                BlockVector neighbour1(lastPos.x + i,lastPos.y,lastPos.z + h);
                if (mapManager->checkIsSolid(neighbour1))
                    return false;
            }

            if (j != 0)
            {
                BlockVector neighbour2(lastPos.x,lastPos.y + j,lastPos.z + h);
                if (mapManager->checkIsSolid(neighbour2))
                    return false;
            }

            if ( (i != 0) && (j != 0))
            {
                BlockVector neighbour3(lastPos.x + i,lastPos.y + j,lastPos.z + h);
                if (mapManager->checkIsSolid(neighbour3))
                    return false;
            }
        }

        return true;
    }

    // when the pos has the same z with the the last pos
    // or if want to access the pos 1 unit higher than the last pos
    // check if there are 2 blocks in the 45 degree directions block the way
    // e.g.: the B are blocks, one cannot access C,E,F,G from L
    //    CBE
    //    BLB
    //    FBG
    if (k == 1) // if  want to access higer position
    {
        if (mapManager->checkIsSolid(lastPos.x,lastPos.y,lastPos.z + 1)) // if the block on top is solid
            return false;
    }

    if ((i != 0) && (j != 0) ) // k == 0  or k == 1
    {
        // all the blocks in the two neighbour 45 direction inside the agentHeight will block the way
        for (int h = 0; h < mapManager->getAgentHeight(); h++)
        {
            BlockVector neighbour1(lastPos.x + i,lastPos.y,lastPos.z + h + k );
            if (mapManager->checkIsSolid(neighbour1))
                return false;

            BlockVector neighbour2(lastPos.x,lastPos.y + j,lastPos.z + h + k );
            if (mapManager->checkIsSolid(neighbour2))
                return false;
        }

        return true;
    }


    return true;
}

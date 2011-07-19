/*
 * opencog/spatial/LSMap3DSearchNode.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Troy Huang
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

#include <opencog/spatial/LSMap3DSearchNode.h>
#include <opencog/util/exceptions.h>

using namespace opencog;
using namespace opencog::spatial;

//for comparing different goal estimate heuristics
int LSMap3DSearchNode::heuristic = LSMap3DSearchNode::SPACE3D;

//static class var needs to be defined outside the class declaration
LocalSpaceMap2D *LSMap3DSearchNode::map;

LSMap3DSearchNode::LSMap3DSearchNode(): x(0), y(0) {}

LSMap3DSearchNode::LSMap3DSearchNode(unsigned int px, unsigned int py, double deltaHeight):
        x(px), y(py) {
    if (LSMap3DSearchNode::map) {
        z = map->getTopSurfaceHeightByGridPoint(GridPoint(px, py));
    }
    deltaZ = deltaHeight;
}

LSMap3DSearchNode::LSMap3DSearchNode(spatial::GridPoint gp, double deltaHeight):
        x(gp.first), y(gp.second) {
    if (LSMap3DSearchNode::map) {
        z = (float)map->getTopSurfaceHeightByGridPoint(gp);
    }
    deltaZ = deltaHeight;
}

void LSMap3DSearchNode::setMap(LocalSpaceMap2D *map)
{
    //todo: pass map as pointer? what if changed underneath
    LSMap3DSearchNode::map = map;
}

bool LSMap3DSearchNode::isLegal()
{
    return isLegal(x, y);
}

bool LSMap3DSearchNode::isLegal(unsigned int x, unsigned int y)
{
    spatial::GridPoint dest(x, y);
    spatial::GridPoint src(this->x, this->y);
    return !map->edgeIllegal(src, dest, this->z, deltaZ);
}

double LSMap3DSearchNode::getDestHeight(const spatial::GridPoint& dest) const
{
    spatial::GridPoint src(this->x, this->y);
    return map->getProperDestAltitude(src, dest, this->z, deltaZ); 
}

bool LSMap3DSearchNode::IsSameState(const LSMap3DSearchNode &rhs)
{
    // same state in LocalSpaceMap2D is simply when (x,y) are the same
    if (x == rhs.x && y == rhs.y) {
        return true;
    } else {
        return false;
    }
}

void LSMap3DSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf( str, "Node position : (%d,%d)\n", x, y );
    cout << str;
}

//Heuristic function that estimates the distance from a Node to the Goal.
float LSMap3DSearchNode::GoalDistanceEstimate(const LSMap3DSearchNode &nodeGoal)
{
    if (heuristic == SPACE3D) {
        //3D Distance
        float xd = abs((float)x - (float)nodeGoal.x);
        float yd = abs((float)y - (float)nodeGoal.y);

        float horizon_diagonal = min(xd, yd);
        float horizon_straight = xd + yd;
        float horizon_delta = (/* sqrt(2) */1.41421356 * horizon_diagonal) + (horizon_straight - 2 * horizon_diagonal);
        float vertical_delta = abs((float)nodeGoal.z - (float)z);
        float distance3d = (float)sqrt(horizon_delta * horizon_delta + vertical_delta * vertical_delta);
        //return h_delta + zd;
        return distance3d;
    }

    //TODO: handle invalid heuristic error
    else {
        cout << "Invalid heuristic error - exiting";
        exit(EXIT_FAILURE);
        //throw FatalErrorException();
    }
}

bool LSMap3DSearchNode::IsGoal(const LSMap3DSearchNode &nodeGoal)
{
    // When the altitude distance is within the height of the agent, we think
    // the agent can reach the goal.
    double agentHeight = LSMap3DSearchNode::map->agentHeight();
    if (x == nodeGoal.x && y == nodeGoal.y && std::abs(z - nodeGoal.z) <= agentHeight) {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool LSMap3DSearchNode::GetSuccessors(AStarSearch<LSMap3DSearchNode> *astarsearch,
                                      LSMap3DSearchNode *parent_node )
{

    //int parent_x = -1;
    //int parent_y = -1;
    unsigned int parent_x = map->xDim() + 1;
    unsigned int parent_y = map->yDim() + 1;

    if (parent_node) {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }

    LSMap3DSearchNode NewNode;

    //push each possible move except allowing the search to go backwards
    //horizontal and vertical moves
    if (isLegal(x - 1, y) && !((parent_x == x - 1) && (parent_y == y)) ) {
        NewNode = LSMap3DSearchNode(x - 1, y, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x - 1, y));
        astarsearch->AddSuccessor(NewNode);
    }

    if (isLegal(x, y - 1) && !((parent_x == x) && (parent_y == y - 1)) ) {
        NewNode = LSMap3DSearchNode(x, y - 1, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x, y - 1));
        astarsearch->AddSuccessor(NewNode);
    }

    if (isLegal(x + 1, y) && !((parent_x == x + 1) && (parent_y == y)) ) {
        NewNode = LSMap3DSearchNode(x + 1, y, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x + 1, y));
        astarsearch->AddSuccessor(NewNode);
    }

    if (isLegal(x, y + 1) && !((parent_x == x) && (parent_y == y + 1)) ) {
        NewNode = LSMap3DSearchNode(x, y + 1, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x, y + 1));
        astarsearch->AddSuccessor(NewNode);
    }

    //diagonal moves
    if (isLegal(x + 1, y + 1) && !((parent_x == x + 1) && (parent_y == y + 1)) ) {
        NewNode = LSMap3DSearchNode(x + 1, y + 1, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x + 1, y + 1));
        astarsearch->AddSuccessor(NewNode);
    }

    if (isLegal(x + 1, y - 1) && !((parent_x == x + 1) && (parent_y == y - 1)) ) {
        NewNode = LSMap3DSearchNode(x + 1, y - 1, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x + 1, y - 1));
        astarsearch->AddSuccessor(NewNode);
    }

    if (isLegal(x - 1, y + 1) && !((parent_x == x - 1) && (parent_y == y + 1)) ) {
        NewNode = LSMap3DSearchNode(x - 1, y + 1, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x - 1, y + 1));
        astarsearch->AddSuccessor(NewNode);
    }

    if (isLegal(x - 1, y - 1) && !((parent_x == x - 1) && (parent_y == y - 1)) ) {
        NewNode = LSMap3DSearchNode(x - 1, y - 1, deltaZ);
        NewNode.z = getDestHeight(spatial::GridPoint(x - 1, y - 1));
        astarsearch->AddSuccessor(NewNode);
    }

    return true;
}

// given this node, what does it cost to move to successor.
float LSMap3DSearchNode::GetCost(const LSMap3DSearchNode &successor)
{
    float vertical_delta = (float)abs(successor.z - z);
    //if horizontal or vertical coast is one
    if (x == successor.x || y == successor.y) {
        return sqrt(1.0 + vertical_delta * vertical_delta);
    }
    //else it is diagonal, so cost is sqrt(2), (sqrt(1^2 + 1^2))
    else {
        return sqrt(2.0 + vertical_delta * vertical_delta); //sqrt(2)
    }
}


void LSMap3DSearchNode::setHeuristic(int h)
{
    cout << "\nSetting heuristic: " << h << "\n";
    heuristic = h;
}

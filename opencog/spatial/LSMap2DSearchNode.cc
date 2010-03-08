/*
 * opencog/spatial/LSMap2DSearchNode.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Dan Zwell
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

#include <opencog/spatial/LSMap2DSearchNode.h>
#include <opencog/util/exceptions.h>

using namespace opencog;
using namespace opencog::spatial;

//for comparing different goal estimate heuristics
int LSMap2DSearchNode::heuristic = LSMap2DSearchNode::MANHATTAN;

//static class var needs to be defined outside the class declaration
LocalSpaceMap2D *LSMap2DSearchNode::map;

LSMap2DSearchNode::LSMap2DSearchNode(): x(0), y(0) {}

LSMap2DSearchNode::LSMap2DSearchNode(unsigned int px, unsigned int py):
        x(px), y(py) {}

LSMap2DSearchNode::LSMap2DSearchNode(spatial::GridPoint gp):
        x(gp.first), y(gp.second) {}

void LSMap2DSearchNode::setMap(LocalSpaceMap2D *map)
{
    //todo: pass map as pointer? what if changed underneath
    LSMap2DSearchNode::map = map;
}

bool LSMap2DSearchNode::isLegal(unsigned int x, unsigned int y)
{
    spatial::GridPoint p(x, y);
    return !map->gridIllegal(p);
}

bool LSMap2DSearchNode::isLegal()
{
    return isLegal(x, y);
}

bool LSMap2DSearchNode::IsSameState(const LSMap2DSearchNode &rhs)
{
    // same state in LocalSpaceMap2D is simply when (x,y) are the same
    if (x == rhs.x && y == rhs.y) {
        return true;
    } else {
        return false;
    }
}

void LSMap2DSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf( str, "Node position : (%d,%d)\n", x, y );
    cout << str;
}

//Heuristic function that estimates the distance from a Node to the Goal.
float LSMap2DSearchNode::GoalDistanceEstimate(const LSMap2DSearchNode &nodeGoal)
{

    if (heuristic == MANHATTAN) {
        //Manhattan Distance
        float xd =  abs((float)x - (float)nodeGoal.x) ;
        float yd =  abs((float)y - (float)nodeGoal.y) ;
        return xd + yd;
    }

    else if (heuristic == DIAGONAL) {
        //Diagonal Distance
        float xd =  abs((float)x - (float)nodeGoal.x);
        float yd =  abs((float)y - (float)nodeGoal.y);

        float h_diagonal = min(xd, yd);
        float h_straight = xd + yd;
        return  (/* sqrt(2) */ 1.41421356 * h_diagonal) + (h_straight - 2*h_diagonal);
    }

    //TODO: handle invalid heuristic error
    else {
        cout << "Invalid heuristic error - exiting";
        exit(EXIT_FAILURE);
        //throw FatalErrorException();
    }
}

bool LSMap2DSearchNode::IsGoal(const LSMap2DSearchNode &nodeGoal)
{
    if (x == nodeGoal.x && y == nodeGoal.y) {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool LSMap2DSearchNode::GetSuccessors(AStarSearch<LSMap2DSearchNode> *astarsearch,
                                      LSMap2DSearchNode *parent_node )
{

    //int parent_x = -1;
    //int parent_y = -1;
    unsigned int parent_x = map->xDim() + 1;
    unsigned int parent_y = map->yDim() + 1;

    if (parent_node) {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }

    LSMap2DSearchNode NewNode;

    //push each possible move except allowing the search to go backwards
    //horizontal and vertical moves
    if (isLegal(x - 1, y) && !((parent_x == x - 1) && (parent_y == y)) ) {
        NewNode = LSMap2DSearchNode(x - 1, y);
        astarsearch->AddSuccessor(NewNode);
    }

    if (isLegal(x, y - 1) && !((parent_x == x) && (parent_y == y - 1)) ) {
        NewNode = LSMap2DSearchNode(x, y - 1);
        astarsearch->AddSuccessor(NewNode);
    }

    if ( isLegal( x + 1, y ) && !((parent_x == x + 1) && (parent_y == y)) ) {
        NewNode = LSMap2DSearchNode(x + 1, y);
        astarsearch->AddSuccessor(NewNode);
    }


    if ( isLegal(x, y + 1) && !((parent_x == x) && (parent_y == y + 1)) ) {
        NewNode = LSMap2DSearchNode(x, y + 1);
        astarsearch->AddSuccessor(NewNode);
    }


    //diagonal moves
    if ( isLegal( x + 1, y + 1 ) && !((parent_x == x + 1) && (parent_y == y + 1)) ) {
        NewNode = LSMap2DSearchNode(x + 1, y + 1);
        astarsearch->AddSuccessor(NewNode);
    }

    if ( isLegal(x + 1, y - 1) && !((parent_x == x + 1) && (parent_y == y - 1)) ) {
        NewNode = LSMap2DSearchNode(x + 1, y - 1);
        astarsearch->AddSuccessor(NewNode);
    }

    if ( isLegal( x - 1, y + 1 ) && !((parent_x == x - 1) && (parent_y == y + 1)) ) {
        NewNode = LSMap2DSearchNode(x - 1, y + 1);
        astarsearch->AddSuccessor(NewNode);
    }

    if ( isLegal( x - 1, y - 1 ) && !((parent_x == x - 1) && (parent_y == y - 1)) ) {
        NewNode = LSMap2DSearchNode(x - 1, y - 1);
        astarsearch->AddSuccessor(NewNode);
    }

    return true;
}

// given this node, what does it cost to move to successor.
float LSMap2DSearchNode::GetCost(const LSMap2DSearchNode &successor)
{
    //if horizontal or vertical coast is one
    if (x == successor.x || y == successor.y) {
        return 1.0;
    }
    //else it is diagonal, so cost is sqrt(2), (sqrt(1^2 + 1^2))
    else {
        return 1.41421356; //sqrt(2)
    }
}


void LSMap2DSearchNode::setHeuristic(int h)
{
    cout << "\nSetting heuristic: " << h << "\n";
    heuristic = h;
}

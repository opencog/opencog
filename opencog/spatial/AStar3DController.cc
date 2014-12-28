/*
 * opencog/spatial/AStar3DController.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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


#include <opencog/spatial/stlastar.h>
#include <opencog/spatial/AStar3DController.h>

using namespace opencog;
using namespace opencog::spatial;

AStar3DController::AStar3DController():
        astarsearch(new AStarSearch<MapSearchNode>(MAX_3D_SEARCH_NODES)),
        useApproxSolution(false) {}

AStar3DController::~AStar3DController()
{
    delete astarsearch;
}

void AStar3DController::setMap(Map *map)
{
    MapSearchNode::setMap(map);
}

void AStar3DController::setStartAndGoalStates(MapSearchNode &nodeStart, MapSearchNode &nodeEnd)
{
    astarsearch->SetStartAndGoalStates(nodeStart, nodeEnd);
}

/**
 * Create new astarsearch with same start and goal nodes, for testing and comparing different heuristics
 */
void AStar3DController::resetSearch(MapSearchNode &startNode, MapSearchNode &goalNode)
{
    delete astarsearch;

    astarsearch = new AStarSearch<MapSearchNode>(MAX_3D_SEARCH_NODES);
    astarsearch->SetStartAndGoalStates(startNode, goalNode);
}

//TODO: check for no map or no start/goal set conditions,
//  check for illegal start/goal (possibly from astarsearch searchstate code)
unsigned int AStar3DController::findPath()
{
    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do {
        SearchState = astarsearch->SearchStep();
        SearchSteps++;

    } while ( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

    if ( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED ) {
        cout << "Search found goal state\n";

        int steps = 0;

        MapSearchNode *node = astarsearch->GetSolutionStart();
#if DISPLAY_SOLUTION
        cout << "Displaying solution\n";
        node->PrintNodeInfo();
#endif

        while (true) {
            node = astarsearch->GetSolutionNext();
            if (!node) {
                break;
            }
            steps ++;
#if DISPLAY_SOLUTION
            node->PrintNodeInfo();
#endif
        };
        cout << "Solution steps " << steps << endl;
#if PRINT_MAP
        addSolutionToMap();
#endif

        // Once you're done with the solution you can free the nodes up
        astarsearch->FreeSolutionNodes();
    }

    else if ( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) {
        cout << "Search terminated. Did not find goal state\n";

        useApproxSolution = true;

        approxSolution = astarsearch->GetApproxOptimalSolution();
    }

    // Display the number of loops the search went through
    cout << "SearchSteps : " << SearchSteps << "\n";

    astarsearch->EnsureMemoryFreed();

    return SearchState;
} //function findPath()

void AStar3DController::debugLists(unsigned int SearchSteps)
{
    cout << "Steps:" << SearchSteps << "\n";

    int len = 0;
    cout << "Open:\n";
    MapSearchNode *p = astarsearch->GetOpenListStart();
    while ( p ) {
        len++;
#if !DEBUG_LIST_LENGTHS_ONLY
        ((MapSearchNode *)p)->PrintNodeInfo();
#endif
        p = astarsearch->GetOpenListNext();
    }

    cout << "Open list has " << len << " nodes\n";

    len = 0;
    cout << "Closed:\n";
    p = astarsearch->GetClosedListStart();
    while ( p ) {
        len++;
#if !DEBUG_LIST_LENGTHS_ONLY
        p->PrintNodeInfo();
#endif
        p = astarsearch->GetClosedListNext();
    }

    cout << "Closed list has " << len << " nodes\n";

}

vector<GridPoint> AStar3DController::getSolutionGridPoints()
{
    vector<GridPoint> solution_points;
    GridPoint gp;
    MapSearchNode *node = astarsearch->GetSolutionStart();

    gp.first = node->x;
    gp.second = node->y;
    solution_points.push_back(gp);

    while (true) {
        node = astarsearch->GetSolutionNext();
        if (!node) {
            break;
        }

        gp.first = node->x;
        gp.second = node->y;
        solution_points.push_back(gp);
    };

    return solution_points;
}

vector<spatial::Point3D> AStar3DController::getSolutionPoints()
{
    vector<spatial::Point3D> solution_points;
    spatial::GridPoint gp;
    Map *map = MapSearchNode::map;

    if (!useApproxSolution) {
        MapSearchNode *previous_node, *node;
        previous_node = node = astarsearch->GetSolutionStart();

        while (true) {
            node = astarsearch->GetSolutionNext();
            if (!node) {
                break;
            }

            gp.first = previous_node->x;
            gp.second = previous_node->y;

            spatial::Point point2d;

            if (map->gridIllegal(gp)) {
                // Adjust the precise coordinate of obstacle.
                spatial::ObjectID id = map->getTallestObjectInGrid(gp);
                point2d = map->centerOf(id);
            } else {
                // If not an obstacle, just get its unsnapped position.
                point2d = map->unsnap(gp);
            }

            //@note the height here is *not* the altitude, but indeed the difference
            //of altitudes between two continuous points. 
            double height = node->z - previous_node->z;
            spatial::Point3D point3d(point2d.first, point2d.second, height);

            solution_points.push_back(point3d);
            previous_node = node;
        };

        // Push back the final goal point.
        gp.first = previous_node->x;
        gp.second = previous_node->y;
        spatial::Point point2d = map->unsnap(gp);
        // We never need to jump at the goal point. 
        spatial::Point3D goal(point2d.first, point2d.second, 0.0);
        solution_points.push_back(goal);
    } else if (!approxSolution.empty()) {
        std::vector<MapSearchNode>::const_iterator prev_node, node;
        prev_node = node = approxSolution.begin();
        for (++node; node != approxSolution.end(); ++node) {
            gp.first = prev_node->x;
            gp.second = prev_node->y;

            spatial::Point point2d;

            if (map->gridIllegal(gp)) {
                // Adjust the precise coordinate of obstacle.
                spatial::ObjectID id = map->getTallestObjectInGrid(gp);
                point2d = map->centerOf(id);
            } else {
                // If not an obstacle, just get its unsnapped position.
                point2d = map->unsnap(gp);
            }

            //@note the height here is *not* the altitude, but indeed the difference
            //of altitudes between two continuous points. 
            double height = node->z - prev_node->z;
            spatial::Point3D point3d(point2d.first, point2d.second, height);

            solution_points.push_back(point3d);
            prev_node = node;
        }
        // Push back the final goal point.
        gp.first = prev_node->x;
        gp.second = prev_node->y;
        spatial::Point point2d = map->unsnap(gp);
        // We never need to jump at the goal point. 
        spatial::Point3D goal(point2d.first, point2d.second, 0.0);
        solution_points.push_back(goal);
        useApproxSolution = false;
    }

    return solution_points;
}

vector<spatial::Point3D> AStar3DController::getShortestCalculatedPath()
{

    vector<spatial::Point3D>  calculatedPath = getSolutionPoints();
    vector<spatial::Point3D>  shortestCalculatedPath;

    opencog::logger().info("AStar - Shortening action plan. It has %d elements.",
                          calculatedPath.size());

    if (calculatedPath.size() < 2 )
        return calculatedPath;

    vector<spatial::Point3D>::iterator it_point = calculatedPath.begin();
    shortestCalculatedPath.push_back( *it_point );
    double alpha = ( - (it_point + 1)->get<1>()) / (it_point->get<0>() - (it_point + 1)->get<0>());
    ++it_point;
    while ( (it_point + 1) != calculatedPath.end() ) {
        double new_alpha = (it_point->get<1>() - (it_point + 1)->get<1>()) / (it_point->get<0>() - (it_point + 1)->get<0>());
        if (std::abs(it_point->get<2>()) > 0.0) {
            // If we need to jump at this point, then it should be stored.
            shortestCalculatedPath.push_back( *it_point );
            alpha = ( - (it_point + 1)->get<1>()) / (it_point->get<0>() - (it_point + 1)->get<0>());
        } else if (abs(new_alpha - alpha) > 0.002 ) {
            shortestCalculatedPath.push_back( *it_point );
            alpha = new_alpha;
        }
        ++it_point;
    }
    shortestCalculatedPath.push_back( *it_point );

    opencog::logger().info(" AStar - Shortening action plan complete. It has %d elements.",
                          shortestCalculatedPath.size());
    return shortestCalculatedPath;
}

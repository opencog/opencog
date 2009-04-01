/*
 * opencog/spatial/AStarController.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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

#include "stlastar.h"
#include "AStarController.h"

using namespace Spatial;

AStarController::AStarController():
    astarsearch(new AStarSearch<MapSearchNode>(MAX_SEARCH_NODES)) {}

AStarController::~AStarController() {
	  delete astarsearch;
}

void AStarController::setMap(Map *map) {
	MapSearchNode::setMap(map);
}

void AStarController::setStartAndGoalStates(MapSearchNode &nodeStart, MapSearchNode &nodeEnd) {
	astarsearch->SetStartAndGoalStates(nodeStart,nodeEnd);
}


/**
 * Create new astarsearch with same start and goal nodes, for testing and comparing different heuristics
 */
void AStarController::resetSearch(MapSearchNode &startNode, MapSearchNode &goalNode) {
    delete astarsearch;

    astarsearch = new AStarSearch<MapSearchNode>(MAX_SEARCH_NODES);
    astarsearch->SetStartAndGoalStates(startNode,goalNode);
}

//TODO: check for no map or no start/goal set conditions, 
//		check for illegal start/goal (possibly from astarsearch searchstate code)
unsigned int AStarController::findPath() {
		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do {
			SearchState = astarsearch->SearchStep();
			SearchSteps++;

#if DEBUG_LISTS  //TODO make this a class method
      debugLists(SearchSteps);
#endif  //DEBUG_LISTS

		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

	if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED ) {
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
	}

	// Display the number of loops the search went through
	cout << "SearchSteps : " << SearchSteps << "\n";

    astarsearch->EnsureMemoryFreed();

    return SearchState;
  } //function findPath()


void AStarController::debugLists(unsigned int SearchSteps) {
  cout << "Steps:" << SearchSteps << "\n";

  int len = 0;
  cout << "Open:\n";
  MapSearchNode *p = astarsearch->GetOpenListStart();
  while( p )
  {
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
  while( p )
  {
    len++;
#if !DEBUG_LIST_LENGTHS_ONLY
    p->PrintNodeInfo();
#endif
    p = astarsearch->GetClosedListNext();
  }

  cout << "Closed list has " << len << " nodes\n";

}


vector<Spatial::GridPoint> AStarController::getSolutionGridPoints() {
    vector<Spatial::GridPoint> solution_points;
    Spatial::GridPoint gp;
    MapSearchNode *node = astarsearch->GetSolutionStart();

    gp.first = node->x;
    gp.second = node->y;
    solution_points.push_back(gp);

    while (true) {
      node = astarsearch->GetSolutionNext();
      if(!node) {
          break;
      }
      
      gp.first = node->x;
      gp.second = node->y;
      solution_points.push_back(gp);
    };

    return solution_points;	
}


vector<Spatial::Point> AStarController::getSolutionPoints() {
    vector<Spatial::Point> solution_points;
    Spatial::GridPoint gp;
    MapSearchNode *node = astarsearch->GetSolutionStart();
    Map *map = MapSearchNode::map;

    gp.first = node->x;
    gp.second = node->y;
    solution_points.push_back(map->unsnap(gp));

    while (true) {
      node = astarsearch->GetSolutionNext();
      if(!node) {
          break;
      }

      gp.first = node->x;
      gp.second = node->y;
      solution_points.push_back(map->unsnap(gp));
    };

    return solution_points;
}

vector<Spatial::Point> AStarController::getShortestCalculatedPath(){

    vector<Spatial::Point>  calculatedPath = getSolutionPoints();
    vector<Spatial::Point>  shortestCalculatedPath;
    
    opencog::logger().log(opencog::Logger::INFO, "AStar - Shortening action plan. It has %d elem.",
        calculatedPath.size());
   
    if (calculatedPath.size() < 2 )
        return calculatedPath;
 
    vector<Spatial::Point>::iterator it_point= calculatedPath.begin();
    shortestCalculatedPath.push_back( *it_point );
    double alpha = (it_point->second - (it_point+1)->second)/(it_point->first - (it_point+1)->first); 
    it_point++;
    while( (it_point+1) != calculatedPath.end() )
    {
        double new_alpha = (it_point->second - (it_point+1)->second)/(it_point->first - (it_point+1)->first); 
        if ( abs(new_alpha-alpha) > 0.002 )
        {
            shortestCalculatedPath.push_back( *it_point );
            alpha = new_alpha;
        }
        it_point++;
    }
    shortestCalculatedPath.push_back( *it_point );

    opencog::logger().log(opencog::Logger::INFO," AStar - Shortening action plan complete. It has %d elem.", 
        shortestCalculatedPath.size());
    return shortestCalculatedPath;
}


/*
#if PRINT_MAP
void AStarController::addSolutionToMap() {
    set<Point> solution_points;
    GridPoint gp;
    LSMap2DSearchNode *node = astarsearch->GetSolutionStart();

    gp.first = node->x;
    gp.second = node->y;
    solution_points.insert(lmap.unsnap(gp));

    while (true) {
      node = astarsearch->GetSolutionNext();
      if(!node) {
          break;
      }

      gp.first = node->x;
      gp.second = node->y;
      solution_points.insert(lmap.unsnap(gp));
    };

    //add solution to map
    //add as obstacle and non-obstacle so we can get nice '*'s
    //lmap.addObjectByGridPoints(rng.util_rand(),
    //lmap.addObject(rng.randint(), solution_points.begin(), solution_points.end());
    lmap.addNonObstacle(rng.randint(), solution_points.begin(), solution_points.end());
}
#endif //PRINT_MAP
*/



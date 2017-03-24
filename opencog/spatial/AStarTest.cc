/*
 * opencog/spatial/AStarTest.cc
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


#include <opencog/util/mt19937ar.h>
#include <opencog/util/StringManipulator.h>

#include <opencog/spatial/AStarController.h>
#include <opencog/spatial/LocalSpaceMap2DUtil.h>

const unsigned int MAP_WIDTH = 200;
const unsigned int MAP_HEIGHT = 200;
const unsigned int OBJECTS = 20; //for randomly populating map
const unsigned int NUM_SEARCHES = 5;
const unsigned int PRINT_MAP = 1;
const unsigned int PRINT_SOLUTION = 0;


using namespace std;

using namespace opencog;
using namespace opencog::spatial;

typedef LocalSpaceMap2D Map;
//typedef LocalSpaceMap2D<unsigned int, double, boost::hash<unsigned int>, NoMetaData > Map;
typedef LSMap2DSearchNode MapSearchNode;
//typedef void* VoidMetaData;
//typedef spatial::GridPoint GridPoint;
//typedef spatial::Point Point;

Map* populateRandomMap()
{
    Map *lmap = new Map(0, MAP_WIDTH, MAP_WIDTH,
                        0, MAP_HEIGHT, MAP_HEIGHT,
                        //5);
                        2);

    cout << "Builing random map objects...\n";
    //populateRandom<unsigned int,double,boost::hash<unsigned int> >(*lmap, OBJECTS,
    populateRandom(*lmap, OBJECTS, spatial::GridPoint(0, 0));
    cout << " done.\n";

    return lmap;
}


//generate a random legal (non-obstacle) node
MapSearchNode getRandomNode(Map *map)
{
    RandGen &rng = randGen();
    MapSearchNode node;
    do {
        node.x = rng.randint(map->xDim());
        node.y = rng.randint(map->yDim());
    } while (!node.isLegal());
    return node;
}

void generateRandomStartAndGoal(Map *lmap, LSMap2DSearchNode &nodeStart, LSMap2DSearchNode &nodeEnd)
{
    // Create a start and goal state with legal nodes
    nodeStart = getRandomNode(lmap);
    nodeEnd = getRandomNode(lmap);
}


//add solution path to the map (as non-obstacle),
//so we can print it out with the map
void addSolutionToMap(AStarController &asc, Map *map)
{
    vector<spatial::Point> solution_points = asc.getSolutionPoints();
    //add solution to map
    spatial::ObjectMetaData no_meta;
//    map->addNonObstacle(toString(rng.randint()), solution_points.begin(), solution_points.end(), no_meta);
    map->addObject(opencog::toString(randGen().randint()), no_meta, false );
}


void printSolution(vector<spatial::GridPoint> points)
{
    cout << "\nGrid Points Solution: \n";
    vector<spatial::GridPoint>::iterator iter;
    int stepCount = 0;
    for (iter = points.begin();
            iter != points.end();
            ++iter) {
        cout << "Node " << stepCount << ": (" << (*iter).first << "," << (*iter).second << ")\n";
        stepCount++;
    }
}

void printPointsSolution(vector<spatial::Point> points)
{
    cout << "\nPoints Solution: \n";
    vector<spatial::Point>::iterator iter;
    int stepCount = 0;
    for (iter = points.begin();
            iter != points.end();
            ++iter) {
        cout << "Node " << stepCount << ": (" << (*iter).first << "," << (*iter).second << ")\n";
        stepCount++;
    }
}


//print out the map with solution path(s) as non-obstacles
void mapout(Map *map)
{
    // Ascii output:
    // Obstacles:             x    (1)
    // non-obstacle objects:  #    (2)  //using non-obstacle object to show solution paths
    // obstacle padding:      |    (4)
    char characters[8]; characters[0] = ' ';

    characters[1] = 'x'; // obstacle
    characters[2] = '#'; // non-obstacle object
    characters[4] = '|'; // obstacle padding
    // Combine symbols for a more informative map output:
    characters[3] = '*'; // obstacle and non-obstacle
    characters[6] = '+'; // padding and non-obstacle
    // If a position is occupied by an obstacle, being padded is insignificant:
    characters[5] = characters[1];
    characters[7] = characters[4];

    for (unsigned int i = 0;i < map->yDim();++i) {
        for (unsigned int j = 0;j < map->xDim();++j) {
            int num = 0;
            if (map->gridOccupied(j, i))
                num += 1;
            if (map->gridOccupied_nonObstacle(j, i))
                num += 2;
//      if (map->gridPadded(j,i))
//        num += 4;
            cout << characters[num];
        }
        cout << std::endl;
    }
}


int main(int argc, char * argv[])
{

    AStarController asc;
    MapSearchNode nodeStart, nodeEnd;

    //seed random number generator
    randGen().seed(time(0));

    //create and populate random map for test purposes
    Map *map = populateRandomMap();
    asc.setMap(map);

    unsigned int SearchCount = 0;
    unsigned int SearchResult;
    while (SearchCount < NUM_SEARCHES) {

        //generate random start and goal for test purposes
        generateRandomStartAndGoal(map, nodeStart, nodeEnd);

        //set start and goal nodes
        asc.setStartAndGoalStates(nodeStart, nodeEnd);
        cout << "\n\nStart: (" << nodeStart.x << "," << nodeStart.y << ")\n";
        cout << "End:   (" << nodeEnd.x << "," << nodeEnd.y << ")\n";

        //find the shortest path
        SearchResult = asc.findPath();
        if (PRINT_MAP) {
            addSolutionToMap(asc, map);
        }

        //get the solution path as grid points
        vector<spatial::GridPoint> solutionGridPoints = asc.getSolutionGridPoints();

        if (PRINT_SOLUTION) {
            printSolution(solutionGridPoints);
        }

        //get the solution path as distance points
        //vector<spatial::Point> solutionPoints = asc.getSolutionPoints();
        //printPointsSolution(solutionPoints);

        SearchCount ++;
    }
    if (PRINT_MAP) {
        mapout(map);
    }

    return (SearchResult == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED ? 0 : 1);
}

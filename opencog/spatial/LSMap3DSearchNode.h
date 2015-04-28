/*
 * opencog/spatial/LSMap3DSearchNode.h
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

/**
 * LSMap3DSearchNode.h
 *
 * User state class for LocalSpaceMap2D used by A* 3D pathfinding
 *
 */
#ifndef LSMAP3DSEARCHNODE_H
#define LSMAP3DSEARCHNODE_H


#include <opencog/spatial/stlastar.h>
#include <opencog/spatial/LocalSpaceMap2D.h>


namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        //typedef LocalSpaceMap2D<unsigned int, double, boost::hash<unsigned int> > Map;
        //typedef LocalSpaceMap2D<> Map;

        class LSMap3DSearchNode
        {

        public:

            unsigned int x;  // the (x,y) grid positions of the node
            unsigned int y;

            double z;  // the height of object in the grid position
            double deltaZ; // the maximum delat height that allows next node to be
                          // in the path.

            //typedef LocalSpaceMap2D<unsigned int, double, boost::hash<unsigned int> > Map;
            typedef LocalSpaceMap2D Map;

            LSMap3DSearchNode();
            LSMap3DSearchNode(unsigned int px, unsigned int py, double deltaHeight);
            LSMap3DSearchNode(GridPoint, double deltaHeight);

            static void setMap(Map *map);

            /**
             * Estimate the distance to goal from current node.
             */
            float GoalDistanceEstimate(const LSMap3DSearchNode &nodeGoal);
            bool IsGoal(const LSMap3DSearchNode &nodeGoal );
            bool GetSuccessors(AStarSearch<LSMap3DSearchNode> *astarsearch, LSMap3DSearchNode *parent_node );
            float GetCost(const LSMap3DSearchNode &successor);
            bool IsSameState(const LSMap3DSearchNode &rhs);
            bool isLegal();
            bool isLegal(unsigned int, unsigned int);
            double getDestHeight(const GridPoint& dest) const;
            static void setHeuristic(int h);
            void PrintNodeInfo();

            enum {
                MANHATTAN,
                DIAGONAL,
                SPACE3D
            };
            static int heuristic;

            //private:
            static Map *map;

        }; //class

    } // spatial
/** @}*/
} // opencog


#endif // LSMAP3DSEARCHNODE_H

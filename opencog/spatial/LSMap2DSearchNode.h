/*
 * opencog/spatial/LSMap2DSearchNode.h
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

/**
 * LSMap2DSearchNode.h
 *
 * User state class for LocalSpaceMap2D used by A* pathfinding
 *
 */
#ifndef _SPATIAL_LSMAP2DSEARCHNODE_H_
#define _SPATIAL_LSMAP2DSEARCHNODE_H_

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

        class LSMap2DSearchNode
        {

        public:

            unsigned int x;  // the (x,y) grid positions of the node
            unsigned int y;

            //typedef LocalSpaceMap2D<unsigned int, double, boost::hash<unsigned int> > Map;
            typedef LocalSpaceMap2D Map;

            LSMap2DSearchNode();
            LSMap2DSearchNode(unsigned int px, unsigned int py);
            LSMap2DSearchNode(GridPoint);

            static void setMap(Map *map);
            float GoalDistanceEstimate(const LSMap2DSearchNode &nodeGoal);
            bool IsGoal(const LSMap2DSearchNode &nodeGoal );
            bool GetSuccessors(AStarSearch<LSMap2DSearchNode> *astarsearch, LSMap2DSearchNode *parent_node );
            float GetCost(const LSMap2DSearchNode &successor);
            bool IsSameState(const LSMap2DSearchNode &rhs);
            bool isLegal(unsigned int x, unsigned int);
            bool isLegal();
            static void setHeuristic(int h);
            void PrintNodeInfo();

            enum {
                MANHATTAN,
                DIAGONAL
            };
            static int heuristic;

            //private:
            static Map *map;

        }; //class

    } // spatial
/** @}*/
} // opencog


#endif // _SPATIAL_LSMAP2DSEARCHNODE_H_

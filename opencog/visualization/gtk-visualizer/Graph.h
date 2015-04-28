/*
 * opencog/visualizer/include/Graph.h
 *
 * Copyright (C) 2012 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Erwin Joosten <eni247@gmail.com>
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

#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <ctime>
#include <math.h>

#include "Vertex.h"
#include "Vertices.h"
#include "Positions.h"
#include "AtomSpaceInterface.h"

using namespace std;

class Graph
{
    public:
		static const int borderWidth=2;

        Vertices vertices;
        Positions positions;

        Graph(AtomSpaceInterface* atomSpaceInterface1);
        virtual ~Graph();

        void AddVertices(vector<Vertex*> vertices);
        void RemoveVertex(Vertex* vertex);
        bool OptimizeLayout();
           //Try to move all vertices in the four directions to see if it
            //reduces the forces (attractive forces from connected vertices
            //and repulsive forces from nearby unconnected vertices) on them.
            //This can take a while if the number of vertices is large so it
            //runs for 20ms at a time and is called 50 times per second.
        void ExpandVertex(Vertex* vertex, int expansionDepth,NodeFilter* nodeFilter,LinkFilter* linkFilter);
           //Recursively retrieve connected atoms from atomspace and add them
           //to the graph.
        void CollapseVertex(Vertex* vertex);

     private:
        AtomSpaceInterface* atomSpaceInterface;
        bool MoveVertexIfItReducesForce(Vertex* vertex, int row, int col, int otherRow, int otherCol, double& force);
};

#endif // GRAPH_H

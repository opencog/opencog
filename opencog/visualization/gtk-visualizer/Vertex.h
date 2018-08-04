/*
 * opencog/visualizer/include/Vertex.h
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

#ifndef VERTEX_H
#define VERTEX_H

#include <string>
#include <vector>
#include <limits>

#include "Positions.h"

using namespace std;

class Positions;

// UUID == Universally Unique Identifier (handle)
typedef unsigned long UUID;

class Vertex
{
    public:
		static const int repulsiveForceRadius = 10;
		static const int repulsiveForceRadiusToTheFourth = repulsiveForceRadius * repulsiveForceRadius * repulsiveForceRadius * repulsiveForceRadius;
        static const int attractiveForceStrength=18;

        vector<Vertex*> connectedVertices;
        vector<string> connectedHandles;
        int row;
        int col;
        string name;
        bool isExpanded;
        bool positionLocked;
        bool isNode; //true=node, false=link
        bool isEllipsis; //dummy atom when there are too many connected atoms to display
        bool isEllipsisClicked; //true if the ellipsis of this atom was double clicked

        int type;
        short STI;
        short LTI;
        double confidenceValue;
        double strength;
        UUID uuid; //unique identifier of the atom (handle)
        string truthValue;

        Vertex();
        virtual ~Vertex();

        void CopyVertex(Vertex &vertex);
        double CalculateForce(Positions *positions);
        static double DistanceSquared(int row, int col, Vertex *otherVertex);
        void Reset();
        Vertex *FindConnectedVertexByUUID(UUID uuid);
        void ConnectVertex(Vertex *vertex2);
        void DisconnectVertex(Vertex *vertex2);
};

#endif // VERTEX_H

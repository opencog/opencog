/*
 * opencog/visualizer/src/Vertex.cpp
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

#include "Vertex.h"

Vertex::Vertex()
{
    row=-999999999;
    col = -999999999;
	name="";
    isExpanded = false;
    positionLocked = false;
    isNode=true;
    isEllipsis=false;
    isEllipsisClicked=false;
    type=0;
    STI =-numeric_limits<short>::max();;
    LTI=-numeric_limits<short>::max();;
    confidenceValue=-numeric_limits<double>::max();;
    strength=-numeric_limits<double>::max();;
    uuid=0;
    truthValue="";
}

Vertex::~Vertex()
{
}

void Vertex::CopyVertex(Vertex &vertex)
{
	name=vertex.name;
    isNode=vertex.isNode;
    type=vertex.type;
    STI =vertex.STI;
    LTI=vertex.LTI;
    confidenceValue=vertex.confidenceValue;
    strength=vertex.strength;
    uuid=vertex.uuid;
    truthValue=vertex.truthValue;
	connectedHandles.insert(connectedHandles.begin(), vertex.connectedHandles.begin(), vertex.connectedHandles.end());
}

double Vertex::CalculateForce(Positions *positions)
{
    //TODO calculate direction of the force so we don't have to try the four directions?
	//TODO if the force is large, move more than one position?
    double force = 0;

    //calculate attractive forces
    for (int i=0; i<connectedVertices.size(); i++)
    {
        double distanceSquared = DistanceSquared(row, col, connectedVertices[i]);
        force += distanceSquared * attractiveForceStrength;
    }

    //calculate repulsive forces
    int minRow = row - repulsiveForceRadius;
    if (minRow < 0)
        minRow = 0;
    int maxRow = row + repulsiveForceRadius;
    if (maxRow >= positions->maxRow)
        maxRow = positions->maxRow - 1;
    int minCol = col - repulsiveForceRadius;
    if (minCol < 0)
        minCol = 0;
    int maxCol = col + repulsiveForceRadius;
    if (maxCol >= positions->maxCol)
        maxCol = positions->maxCol - 1;

    for (int r = minRow; r <= maxRow; r++)
        for (int c = minCol; c <= maxCol; c++)
        {
            Vertex *otherVertex = positions->positions[r][c];
            if (otherVertex != NULL)
            {
                if (otherVertex->row != row || otherVertex->col != col)
                {
                    double distanceSquared = DistanceSquared(row, col, otherVertex);
                    force += 1 / distanceSquared * repulsiveForceRadiusToTheFourth;
                }
                else
                {
                    if (otherVertex == this)
                    { }//no influence on force
                    else
                        force += repulsiveForceRadiusToTheFourth;
                }
            }
        }

    return force;
}

void Vertex::ConnectVertex(Vertex *vertex2)
{
    connectedVertices.push_back(vertex2);
    vertex2->connectedVertices.push_back(this);
}

void Vertex::DisconnectVertex(Vertex *vertex2)
{
    for(int i=0;i<connectedVertices.size();i++)
        if(connectedVertices[i]==vertex2)
        {
            connectedVertices.erase(connectedVertices.begin()+i);
            return;
        }
}

double Vertex::DistanceSquared(int row, int col, Vertex *otherVertex)
{
    double dRow = (double)otherVertex->row - (double)row;
    double dCol = (double)otherVertex->col - (double)col;
    return dRow * dRow + dCol * dCol;
}

Vertex *Vertex::FindConnectedVertexByUUID(UUID uuid)
{
    for(int i=0;i<connectedVertices.size();i++)
        if(connectedVertices[i]->uuid==uuid)
            return connectedVertices[i];
    return NULL;
}

void Vertex::Reset()
{
    connectedVertices.clear();
    isExpanded = false;
    positionLocked = false;
}


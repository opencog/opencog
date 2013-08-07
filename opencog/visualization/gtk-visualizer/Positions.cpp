/*
 * opencog/visualizer/src/Positions.cpp
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

#include "Positions.h"

Positions::Positions()
{
    for (int r = 0; r < maxRow; r++)
          for (int c = 0; c < maxCol; c++)
        	  positions[r][c]=NULL;
}

Positions::~Positions()
{

}

Vertex* Positions::FindClosestVertex(int row, int col)
{
    const int radius = 3;
    int minR = row - radius;
    if (minR < 0)
        minR = 0;
    int maxR = row + radius;
    if (maxR >= maxRow)
        maxR = maxRow - 1;
    int minC = col - radius;
    if (minC < 0)
        minC = 0;
    int maxC = col + radius;
    if (maxC >= maxCol)
        maxC = maxCol - 1;

    double leastDistanceSquared = 9999999.0;
    Vertex* closestVertex = NULL;
    for (int r = minR; r <= maxR; r++)
        for (int c = minC; c <= maxC; c++)
        {
            Vertex* otherVertex = positions[r][c];
            if (otherVertex != NULL)
            {
                double distanceSquared = Vertex::DistanceSquared(row, col, otherVertex);
                if (distanceSquared < leastDistanceSquared)
                {
                    leastDistanceSquared = distanceSquared;
                    closestVertex = otherVertex;
                }
            }
        }
    return closestVertex;
}

void Positions::FindNearestFreePosition(int row, int col, int* freeRow, int* freeCol)
{ //Try to find a free position near a vertex by scanning in ever
  //larger squares around the vertex.

    *freeRow = -1;
    *freeCol = -1;
    for (int radius = 3; radius < maxCol; radius+=2)
    {
        int minR = row - radius;
        if (minR < 0)
            minR = 0;
        int maxR = row + radius;
        if (maxR >= maxRow)
            maxR = maxRow - 1;
        int minC = col - radius;
        if (minC < 0)
            minC = 0;
        int maxC = col + radius;
        if (maxC >= maxCol)
            maxC = maxCol - 1;

        for (int c = minC; c <= maxC; c++)
        {
            if (positions[minR][c] == NULL)
            {
                *freeRow = minR;
                *freeCol = c;
                return;
            }
            if (positions[maxR][c] == NULL)
            {
                *freeRow = maxR;
                *freeCol = c;
                return;
            }
        }

        for (int r = minR + 1; r <= maxR - 1; r++)
        {
            if (positions[r][minC] == NULL)
            {
                *freeRow = r;
                *freeCol = minC;
                return;
            }
            if (positions[r][maxC] == NULL)
            {
                *freeRow = r;
                *freeCol = maxC;
                return;
            }
        }

        if (minR == 0 && maxR == maxRow - 1 && minC == 0 && maxC == maxCol - 1)
        {
            throw runtime_error("There are too many atoms to find a free position for all of them.");
        }
    }
}
Vertex* Positions::GetAt(int row, int col)
{
    return positions[row][col];
}

void Positions::MoveTo(Vertex* vertex, int row, int col)
{
    positions[vertex->row][vertex->col] = NULL;
    positions[row][col] = vertex;
    vertex->row = row;
    vertex->col = col;
}

void Positions::PlaceAt(Vertex* vertex, int row, int col)
{
    positions[row][col] = vertex;
    vertex->row = row;
    vertex->col = col;
}

void Positions::RemoveAt(int row, int col)
{
    if (positions[row][col] != NULL)
    {
        positions[row][col]->row = -1;
        positions[row][col]->col = -1;
    }
        positions[row][col] = NULL;
}


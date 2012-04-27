/*
 * opencog/visualizer/include/Positions.h
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

#ifndef POSITIONS_H
#define POSITIONS_H

#include "../include/Vertex.h"
#include <exception>
#include <stdexcept>

class Vertex;

class Positions
{
    public:
		static const int maxRow=70;
		static const int maxCol=100;
        Vertex* positions[maxRow][maxCol];

        Positions();
        virtual ~Positions();

        Vertex *GetAt(int row, int col);
        void PlaceAt(Vertex* vertex, int row, int col);
        void MoveTo(Vertex* vertex, int row, int col);
        void RemoveAt(int row, int col);
        Vertex *FindClosestVertex(int row, int col);
        void FindNearestFreePosition(int row, int col, int *freeRow, int *freeCol);
          //Try to find a free position near a vertex by scanning in ever
          //larger squares around the vertex.

    private:
};

#endif // POSITIONS_H

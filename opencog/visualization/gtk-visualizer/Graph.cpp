/*
 * opencog/visualizer/src/Graph.cpp
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

#include "Graph.h"

Graph::Graph(AtomSpaceInterface* atomSpaceInterface1)
{
    atomSpaceInterface=atomSpaceInterface1;
}

Graph::~Graph()
{
}

void Graph::AddVertices(vector<Vertex*> newVertices)
{
	double rowsDouble=sqrt(newVertices.size());
	int rows=rowsDouble+0.99;
	int rowSeparation=positions.maxRow/(rows+1);
	int cols=(double)newVertices.size()/rows+0.99;
	int colSeparation=positions.maxCol/(cols+1);
	int i=0;
	for(int row=rowSeparation; row<positions.maxRow; row+=rowSeparation)
		for(int col=colSeparation; col<=positions.maxCol-colSeparation; col+=colSeparation)
		{
			vertices.vertices.push_back(newVertices[i]);
			positions.PlaceAt(newVertices[i], row, col);
			i++;
			if((std::size_t)i==newVertices.size())
			{
				row=positions.maxRow;
				col=positions.maxCol;
			}
		}
	/*
	double separation=(double)positions.maxCol/newVertices.size();
	double col=separation/2;
	for(int i=0;i<newVertices.size();i++)
	{
		vertices.vertices.push_back(newVertices[i]);
		positions.PlaceAt(newVertices[i], positions.maxRow / 2, col);
		col+=separation;
	}
	*/
}

void Graph::RemoveVertex(Vertex* vertex)
{
	positions.RemoveAt(vertex->row, vertex->col);
	vertices.Remove(vertex);
}

void Graph::CollapseVertex(Vertex* vertex)
{   //Remove all vertices from the graph that are only connected to
    //the given vertex.

    std::size_t l = 0;
    while (l < vertex->connectedVertices.size())
    {
        Vertex* otherVertex = vertex->connectedVertices[l];
        if (otherVertex->connectedVertices.size()==1) //only connected to vertex
        {
            positions.RemoveAt(otherVertex->row, otherVertex->col);
            vertices.Remove(otherVertex);
            vertex->connectedVertices.erase(vertex->connectedVertices.begin()+l);
        }
        else
        {
            l++;
        }
    }
    vertex->isExpanded = false;
    vertex->isEllipsisClicked=false;
}

void Graph::ExpandVertex(Vertex* vertex, int expansionDepth,NodeFilter* nodeFilter,LinkFilter* linkFilter)
{   //Recursively retrieve connected atoms from atomspace and add them
    //to the graph.

    if(expansionDepth==0)
        return;

    vector<Vertex*> connectedVertices;
    atomSpaceInterface->GetConnectedAtoms(vertex,nodeFilter,linkFilter,connectedVertices);

    for (std::size_t i = 0; i < connectedVertices.size(); i++)
    {
        Vertex* dependentVertex = connectedVertices[i];

        Vertex* existingVertex =NULL;
        if(!dependentVertex->isEllipsis)
        	existingVertex =vertices.FindVertexByUUID(dependentVertex->uuid);

        if (existingVertex == NULL)
        {
            vertices.vertices.push_back(dependentVertex);

            int freeRow;
            int freeCol;
            positions.FindNearestFreePosition(vertex->row, vertex->col, &freeRow, &freeCol);
            positions.PlaceAt(dependentVertex, freeRow, freeCol);

            vertex->ConnectVertex(dependentVertex);

            if(!dependentVertex->isEllipsis)
            	ExpandVertex(dependentVertex, expansionDepth - 1, nodeFilter, linkFilter);
        }
        else //clone of connectedVertex already part of the graph
        {
            Vertex* connectedVertex=vertex->FindConnectedVertexByUUID(dependentVertex->uuid);
            if (connectedVertex == NULL)
            {
                vertex->ConnectVertex(existingVertex);
                delete dependentVertex;
            }
            else //already connected
            {
                delete dependentVertex;
            }
        }
    }

    vertex->isExpanded = true; //TODO only change to true if new vertices were added
}

bool Graph::MoveVertexIfItReducesForce(Vertex* vertex, int row, int col, int otherRow, int otherCol, double &force)
{
    bool isMoved = false;
    Vertex* otherVertex = positions.positions[otherRow][otherCol];
    if (otherVertex == NULL)
    {
        positions.MoveTo(vertex, otherRow, otherCol);
        double newForce = vertex->CalculateForce(&positions);
        if (newForce < force)
        {
            isMoved = true; //keep new position
            force=newForce;
        }
        else
        {
            positions.MoveTo(vertex, row, col); //restore position
        }
    }
    return isMoved;
}

bool Graph::OptimizeLayout()
{

    bool anyMoves = false;
    unsigned int startClock = clock();

    do
    {
        anyMoves = false;
        for (std::size_t i = 0; i < vertices.vertices.size(); i++)
        {
            Vertex* vertex = vertices.vertices[i];
            if (!vertex->positionLocked)
            {
                double force = vertex->CalculateForce(&positions);
                bool isMovedHorizontally = false;

                int row = vertex->row;
                int col = vertex->col;
                if (col > borderWidth)
                {
                    int newRow = row;
                    int newCol = col - 1;
                    isMovedHorizontally = MoveVertexIfItReducesForce(vertex, row, col, newRow, newCol, force);
                }
                if (!isMovedHorizontally)
                {
                    if (col < positions.maxCol - borderWidth - 1)
                    {
                        int newRow = row;
                        int newCol = col + 1;
                        isMovedHorizontally = MoveVertexIfItReducesForce(vertex, row, col, newRow, newCol, force);
                    }
                }

                bool isMovedVertically = false;
                row = vertex->row;
                col = vertex->col;
                if (row > borderWidth)
                {
                    int newRow = row - 1;
                    int newCol = col;
                    isMovedVertically = MoveVertexIfItReducesForce(vertex, row, col, newRow, newCol, force);
                }
                if (!isMovedVertically)
                {
                    if (row < positions.maxRow - borderWidth - 1)
                    {
                        int newRow = row + 1;
                        int newCol = col;
                        isMovedVertically = MoveVertexIfItReducesForce(vertex, row, col, newRow, newCol, force);
                    }
                }

                if (isMovedHorizontally || isMovedVertically)
                    anyMoves = true;
            }
        }
    }
    while (anyMoves && clock()-startClock < 50);
    return anyMoves;
}


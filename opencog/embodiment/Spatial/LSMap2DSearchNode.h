/**
 * LSMap2DSearchNode.h
 * 
 * User state class for LocalSpaceMap2D used by A* pathfinding
 * 
 */
#ifndef LSMAP2DSEARCHNODE_H
#define LSMAP2DSEARCHNODE_H

#include "stlastar.h"
#include "LocalSpaceMap2D.h"


namespace Spatial {

//typedef LocalSpaceMap2D<unsigned int, double, boost::hash<unsigned int> > Map;	
//typedef LocalSpaceMap2D<> Map;

class LSMap2DSearchNode {	

public:
	
	unsigned int x;	 // the (x,y) grid positions of the node
	unsigned int y;

	//typedef LocalSpaceMap2D<unsigned int, double, boost::hash<unsigned int> > Map;	
	typedef LocalSpaceMap2D Map;	
	
	LSMap2DSearchNode();
	LSMap2DSearchNode(unsigned int px, unsigned int py);
	LSMap2DSearchNode(Spatial::GridPoint);
	
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

}  //namespace Spacial


#endif /*LSMAP2DSEARCHNODE_H*/

//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"
#include <limits>


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) {
		std::cout << "\nIn canBeTraversed"<< std::endl;
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id) {
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	double euclidean_distance(Util::Point a, Util::Point b) {
		Util::Point d2 = a*b;
		return std::sqrt(d2.x + d2.z);
	}

	int indexWithLeastF(std::vector<AStarPlannerNode> list) { 
		int min = std::numeric_limits<int>::max();
		int index = -1;
		for (int i = 0; i < list.size(); i++) {
			if (list[i].f <= min) {
				min = list[i].f;
				index = i;
			}
		}
		return i;
	}

	void addNeighborIfGood(AStarPlannerNode parent, std::vector<AStarPlannerNode> neighbors, Util::Point point) {
		if (!gSpatialDatabase -> hasAnyItems(point.x, point.z)) {
			AStarPlannerNode node(point, -1, -1);
			node.parent = parent;
			neighbors.push_back(node);
		}
	}

	std::vector<AStarPlannerNode> getNeighbors(AStarPlannerNode a) { 
		std::vector<AStarPlannerNode> neighbors;
		// Top 
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x, a.point.z+1);

		// Top Right
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x+1, a.point.z+1);

		// Right
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x+1, a.point.z);

		// Right Bottom
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x+1, a.point.z -1);

		// Bottom
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x, a.point.z -1);

		// Bottom Left
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x-1, a.point.z -1);

		// Left
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x-1, a.point.z);

		// Left Top
		addNeighborIfGood(a, neighbors, Util::Point(a.point.x-1, a.point.z+1);
	}

	std::vector<Util::Point> trace(AStarPlannerNode node) {
		std::vector<Util::Point> trace;
		while (node != NULL) {
			trace.push_back(node);
			node = node.parent;
		}
		return trace;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path) {
		gSpatialDatabase = _gSpatialDatabase;
		// Psuedocode from: http://web.mit.edu/eranki/www/tutorials/search/
		// Initialize the open list
		std::vector<AStarPlannerNode> open_list;

		// Initialize the closed list
		std::vector<AStarPlannerNode> closed_list;

		// Put the starting node on the open list
		open_list.push_back(AStarPlannerNode(start, euclidean_distance(start, goal), 0));

		// while openlist is not empty
		while(!open_list.empty()) {
			// find the node with the least f on the open list, call it "q"
			int indexOfQ = indexWithLeastF(open_list);
			AStarPlannerNode q = open_list[indexOfQ];

			// pop q off the open list
			open_list.erase(open_list.begin() + indexOfQ);

			// generate q's 8 successors and set their parents to q
			std::vector<AStarPlannerNode> successors = getNeighbors(q);

			// generate q's 8 successors and set their parents to q
			for (int i = 0; i < successors.size(); i++) {
				// if successor is the goal, stop the search
				if (successors[i].point == goal) {
					agent_path = trace(successors[i]);
					return true;
				}

				// successor.g = q.g + distance between successor and q
				successors[i].g = q.g + euclidean_distance(q.point, successors[i].point);
				
				// successor.h = distance from goal to successor
				successors[i].h = euclidean_distance(goal, successors[i].point);

				// successor.f = successor.g + successor.h
				successors[i].f = successors[i].g + successors[i].h;

				for (int j = 0; j < open_list.size(); j++) {
					// if a node with the same position as successor is in the OPEN list which has a lower f than successor, skip this successor
					if (open_list[j].point == successor[i].point && open_list[j].f < successors[i].f) {
						goto skip_this_successor;
					}
				}

				for (int j = 0; j < closed_list.size(); j++) {
					// if a node with the same position as successor is in the CLOSED list which has a lower f than successor, skip this successor
					if (closed_list[j].point == successor[i].point && closed_list[j].f < successor[i].f) {
						goto skip_this_successor;
					}
				}

				//otherwise, add the node to the open list
				open_list.push_back(successor[i]);
			}
			skip_this_successor:
			// push q on the closed list
			closed_list.push_back(q);
		}

		return false;
	}
}

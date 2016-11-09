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
	std::ostream& operator<<(std::ostream &strm, const AStarPlannerNode &a) {
		return strm << "AStarPlannerNode(" << a.point << ", " << a.g << ", " << a.h << ", " << a.f << ")" << "->" << a.parent;
	}
	
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) {
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

	double AStarPlanner::euclidean_distance(Util::Point a, Util::Point b) {
		return std::sqrt(
			(a.x - b.x) * (a.x - b.x) +
			(a.y - b.y) * (a.y - b.y) + 
			(a.z - b.z) * (a.z - b.z)
			);
	}

	int AStarPlanner::indexWithLeastF(std::vector<AStarPlannerNode*> list) { 
		double min = std::numeric_limits<double>::max();
		int index = -1;
		for (int i = 0; i < list.size(); i++) {
			if (list[i] -> f <= min) {
				min = list[i] -> f;
				index = i;
			}
		}
		return index;
	}

	bool AStarPlanner::addNeighborIfGood(AStarPlannerNode* parent, std::vector<AStarPlannerNode*> &neighbors, Util::Point point) {
		int dbIndex = gSpatialDatabase -> getCellIndexFromLocation(point);

		if (canBeTraversed(dbIndex)) {
			AStarPlannerNode* node = new AStarPlannerNode(point, double(0), double(0), double(0), parent);
			neighbors.push_back(node);
			return true;
		} 

		return false;
	}

	std::vector<AStarPlannerNode*> AStarPlanner::getNeighbors(AStarPlannerNode* a) { 
		std::vector<AStarPlannerNode*> neighbors;
		// Top 
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x, 0,  a -> point.z+1));
		
		// Top Right
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x+1, 0,  a -> point.z+1));
		
		
		// Right
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x+1, 0,  a -> point.z));
		
		
		// Right Bottom
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x+1, 0,  a -> point.z -1));
		
		
		// Bottom
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x, 0,  a -> point.z -1));
		
		
		// Bottom Left
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x-1, 0,  a -> point.z -1));
		
		
		// Left
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x-1, 0,  a -> point.z));
		
		
		// Left Top
		addNeighborIfGood(a, neighbors, Util::Point(a -> point.x-1, 0,  a -> point.z+1));
		return neighbors;
	}

	std::vector<Util::Point> AStarPlanner::trace(AStarPlannerNode* node) {
		std::vector<Util::Point> trace;
		AStarPlannerNode* temp = node;
		trace.push_back(temp -> point);
		while (temp -> parent != NULL) {
			temp = temp -> parent;
			trace.push_back(temp -> point);
		}
		return trace;
	}

	void AStarPlanner::printList(std::vector<AStarPlannerNode*> list) {
		for (int i = 0; i < list.size(); i++) {
			std::cout << *list[i] << std::endl;
		}
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path) {
		gSpatialDatabase = _gSpatialDatabase;
		// Psuedocode from: http://web.mit.edu/eranki/www/tutorials/search/
		// Initialize the open list
		std::vector<AStarPlannerNode*> open_list;

		// Initialize the closed list
		std::vector<AStarPlannerNode*> closed_list;

		// Put the starting node on the open list
		AStarPlannerNode* root = new AStarPlannerNode(start, double(0), double(0), double(0), NULL);
		open_list.push_back(root);

		// while openlist is not empty
		while (!open_list.empty()) {

			// find the node with the least f on the open list, call it "q"
			int indexOfQ = indexWithLeastF(open_list);
			AStarPlannerNode* q = open_list[indexOfQ];


			// pop q off the open list
			open_list.erase(open_list.begin() + indexOfQ);

			// generate q's 8 successors and set their parents to q
			std::vector<AStarPlannerNode*> successors = getNeighbors(q);



			// generate q's 8 successors and set their parents to q
			for (int i = 0; i < successors.size(); i++) {
				// if successor is the goal, stop the search
				if (successors[i] -> point == goal) {
					agent_path = trace(successors[i]);
					return true;
				}

				// successor.g = q.g + distance between successor and q
				successors[i] -> g = q -> g + euclidean_distance(q -> point, successors[i] -> point);
				
				// successor.h = distance from goal to successor
				successors[i] -> h = euclidean_distance(goal, successors[i] -> point);

				// successor.f = successor.g + successor.h
				successors[i] -> f = successors[i] -> g + successors[i] -> h;


				bool skip = false;
				for (int j = 0; j < open_list.size(); j++) {
					// if a node with the same position as successor is in the OPEN list which has a lower f than successor, skip this successor
					if (open_list[j] -> point == successors[i] -> point && open_list[j] -> f < successors[i] -> f) {
						skip = true;
					}
				}

				for (int j = 0; j < closed_list.size(); j++) {
					// if a node with the same position as successor is in the CLOSED list which has a lower f than successor, skip this successor
					if (closed_list[j] -> point == successors[i] -> point && closed_list[j] -> f < successors[i] -> f) {
						skip = true;
					}
				}

				//otherwise, add the node to the open list
				if (!skip) {
					open_list.push_back(successors[i]);
				}
			}
			// push q on the closed list
			closed_list.push_back(q);
		}

		return false;
	}

}

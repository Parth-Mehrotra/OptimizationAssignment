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


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
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



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		for(int i=0; i<agent_path.size; i++){
			std::cout << agent_path[i] << std::endl;
		}

		//1.closedset := empty set
		std::vector<AStarPlannerNode> closedSet;
		//2.openset={start} where start(g=0)
		std::vector<AStarPlannerNode> openSet;
		openSet.push_back(AStarPlannerNode(start,0,0,NULL));

		//make map
		std::vector<std::vector<AStarPlannerNode>> map;

		//value of top-left x
		float topleftx = _gSpatialDatabase->getOriginX();
		float topleftz = _gSpatialDatabase->getOriginZ();
		float bottomrightx = topleftx + _gSpatialDatabase->getGridSizeX();
		float bottomrightz = topleftz + _gSpatialDatabase->getGridSizeZ();

		std::cout <<"topleftx"<< topleftx << std::endl;
		std::cout << "topleftz" << topleftz << std::endl;
		std::cout << "bottomrightx" << bottomrightx << std::endl;
		std::cout << "bottomrightz" << bottomrightz << std::endl;
		
		//set g's and f's of map
		for (int x = topleftx; x < bottomrightx; x++) {
			for (int z = topleftz; z < bottomrightz; z++) {
				//4.g_score=map with default value of infinity
				(map[x])[z].g = INFINITY;
				(map[x])[z].f = INFINITY;
			}
		}
		//(map[start.x])[start.y].g = 0;


		std::vector<Util::Point> tempPath;
		tempPath.push_back(Point(6, 0, -1));
		tempPath.push_back(Point(6, 0, 10));
		tempPath.push_back(Point(-10, 0, 10));
		tempPath.push_back(Point(-10, 0, -1));
		agent_path = tempPath;

		//TODO
		std::cout<<"\nIn A*"<<std::endl;

		return false;
	}
}
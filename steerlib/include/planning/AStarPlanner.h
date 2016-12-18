//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			double h;
			double rhs;
			std::vector<float> key;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode(Util::Point _point, double _g, double _f, double _h, AStarPlannerNode* _parent) {
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
				h = _h;
			}
			bool operator<(AStarPlannerNode other) const {
				return this->f < other.f;
			}
			bool operator>(AStarPlannerNode other) const {
				return this->f > other.f;
			}
			bool operator==(AStarPlannerNode other) const {
				return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
			}

			bool operator!=(AStarPlannerNode other) const {
				return !(*this == other);
			}
	};


	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/

			float w;
			float w_s;
			int placeInCode;
			SteerLib::Clock clockMeasure;
			float maxTime;
			int edgeCostChanges;
			std::vector<AStarPlannerNode*> visitedNodes;


			std::vector<AStarPlannerNode*> open_list;
			std::vector<AStarPlannerNode*> closed_list;
			std::vector<AStarPlannerNode*> incons_list;
			AStarPlannerNode* root;
			AStarPlannerNode* goalNode;



			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/
			bool weightedAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
			float fvalue(AStarPlannerNode *s);
			void improvePath();
			bool ARAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
			
			
			std::vector<float> key(AStarPlannerNode *s);
			void updateStateAD(AStarPlannerNode *s);
			void computeShortestPathAD();
			bool KeyAlessthanB(AStarPlannerNode *s, AStarPlannerNode *s2);
			bool ADStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
		private:
			double euclidean_distance(Util::Point a, Util::Point b);

			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
			int indexWithLeastF(std::vector<AStarPlannerNode*> list);
			int indexWithLeastfValueARA(std::vector<AStarPlannerNode*> list, float w);
			int indexWithLeastghARA(std::vector<AStarPlannerNode*> list);
			bool addNeighborIfGood(AStarPlannerNode* parent, std::vector<AStarPlannerNode*> &neighbors, Util::Point point);
			std::vector<AStarPlannerNode*> getNeighbors(AStarPlannerNode* a);
			std::vector<Util::Point> traceARA(AStarPlannerNode* node);
			std::vector<Util::Point> trace(AStarPlannerNode* node);
			void printList(std::vector<AStarPlannerNode*> list);
	};


}


#endif

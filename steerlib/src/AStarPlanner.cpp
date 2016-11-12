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
		double min = 1000000;// std::numeric_limits<double>::max();
		int index = -1;
		for (int i = 0; i < list.size(); i++) {
			if (list[i] -> f <= min) {
				min = list[i] -> f;
				index = i;
			}
		}
		return index;
	}

	int AStarPlanner::indexWithLeastfValueARA(std::vector<AStarPlannerNode*> list, float w) {
		double min = 1000000;// std::numeric_limits<double>::max();
		int index = -1;
		for (int i = 0; i < list.size(); i++) {
			if (list[i]->g+w*list[i]->h <= min) {
				min = list[i]->g + w*list[i]->h;
				list[i]->f= list[i]->g + w*list[i]->h;
				index = i;
			}
		}
		return index;
	}

	int AStarPlanner::indexWithLeastghARA(std::vector<AStarPlannerNode*> list) {
		double min = 1000000;// std::numeric_limits<double>::max();
		int index = -1;
		for (int i = 0; i < list.size(); i++) {
			if (list[i]->g+ list[i]->h <= min) {
				min = list[i]->g+ list[i]->h;
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
		//std::cout << temp->point << std::endl;
		while (temp -> parent != NULL) {
			temp = temp -> parent;
			trace.push_back(temp -> point);
			//std::cout << temp->point << std::endl;
		}
		std::vector<Util::Point> traceTemp;
		for (int i = 0; i < trace.size(); i++) {
			
			//traceTemp[trace.size() - 1 - i] = trace[i];
			traceTemp.push_back(trace[trace.size() - 1 - i]);
		}
		std::cout << "trace:---------------------" << traceTemp << std::endl;
		return traceTemp;
	}

	void AStarPlanner::printList(std::vector<AStarPlannerNode*> list) {
		for (int i = 0; i < list.size(); i++) {
			std::cout << *list[i] << std::endl;
		}
	}

	bool AStarPlanner::weightedAStar(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path) {
		
		// Psuedocode from: http://web.mit.edu/eranki/www/tutorials/search/
		// Initialize the open list
		std::vector<AStarPlannerNode*> open_list;

		// Initialize the closed list
		std::vector<AStarPlannerNode*> closed_list;

		// Put the starting node on the open list
		AStarPlannerNode* root = new AStarPlannerNode(start, double(0), euclidean_distance(goal, start), euclidean_distance(goal, start), NULL);
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
				successors[i] -> f = successors[i] -> g + w*successors[i] -> h;


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

		return true;
	}


	//ARA* helpers

	float AStarPlanner::fvalue(AStarPlannerNode * s) {
		return s->g + w*s->h;
	}



	void AStarPlanner::improvePath() {
		
		
		//2. minimum of fvalue of s of all s
		int indexOfS = indexWithLeastfValueARA(open_list,w);
		AStarPlannerNode* s= open_list[indexOfS];
		
		//temp print
		
		while (goalNode->f > s->f) {
			
			//3
			open_list.erase(open_list.begin() + indexOfS);
			//4
			closed_list.push_back(s);
			

			//5
			std::vector<AStarPlannerNode*> successors = getNeighbors(s);
			// generate q's 8 successors and set their parents to q
			
			for (int i = 0; i < successors.size(); i++) {
				AStarPlannerNode* s_s = successors[i];
				if (s_s->point == goalNode->point) {
					s_s = goalNode;
					
					
				}
				
				
				//std::cout << "s_s: " <<s_s->point<< std::endl;
				bool skip = false;
				//6. if s' was not visited before then...
				//if not in closed list
				for (int j = 0; j < closed_list.size(); j++) {
					// if a node with the same position as successor is in the CLOSED list, we have visited it before
					if (closed_list[j]->point == s_s->point) {
						skip = true;
					}
				}
				//if not in open list
				for (int j = 0; j < open_list.size(); j++) {
					// if a node with the same position as successor is in the CLOSED list, we have visited it before
					if (open_list[j]->point == s_s->point) {
						skip = true;
					}
				}
				//if not in inconsistent list
				for (int j = 0; j < incons_list.size(); j++) {
					// if a node with the same position as successor is in the CLOSED list, we have visited it before
					if (incons_list[j]->point == s_s->point) {
						skip = true;
					}
				}

				//otherwise, add the node to the open list
				if (!skip) {
					//7. g(s')=inf
					if (s_s->point == goalNode->point) {
						goalNode->g = 100000;
						goalNode->parent = s;
						s_s->g = 100000;
						s_s->parent = s;
					}
					else {
						s_s->g = 100000;
						s_s->parent = s;
					}
				
				}
				//8
				if (s_s->g > s->g + euclidean_distance(s->point, s_s->point)) {
					if (s_s->point == goalNode->point) {
						goalNode->g=s->g + euclidean_distance(s->point, s_s->point);
					}
					s_s->g = s->g + euclidean_distance(s->point, s_s->point);
					skip = false;
					
					for (int j = 0; j < closed_list.size(); j++) {
						// node is in the list
						if (closed_list[j]->point == s_s->point) {
							skip = true;
						}
					}
					if (!skip) {
						if (s_s->point == goalNode->point) {
							goalNode->h = euclidean_distance(s_s->point, goalNode->point);
							goalNode->f = goalNode->g + w*goalNode->h;
							open_list.push_back(goalNode);
						}
						else {
							s_s->h = euclidean_distance(s_s->point, goalNode->point);
							s_s->f = s_s->g + w*s_s->h;
							open_list.push_back(s_s);
						}
						
						//std::cout << "we put in open list with f: " << s_s->g+w*s_s->h << std::endl;
					}
					else {
						s_s->h = euclidean_distance(s_s->point, goalNode->point);
						incons_list.push_back(s_s);
						//std::cout << "we put in incons list " << std::endl;
					}

				}
			}

			//2. minimum of fvalue of s of all s
			indexOfS = indexWithLeastfValueARA(open_list, w);
			
			if (open_list[indexOfS] != goalNode) {
				s = open_list[indexOfS];
			}
			else {
				s = goalNode;
			}

			
			
			//std::cout << "we are printing open list: " << std::endl;
			//printList(open_list);
			//std::cout << "we are printing close list: " << std::endl;
			//printList(closed_list);
			//std::cout << "we are printing incons list: " << std::endl;
			//printList(incons_list);
			int x;
			//std::cin >> x;

		}
		
		
		
		//AStarPlannerNode* temp = new AStarPlannerNode(Util::Point(0,1,2), double(0), double(0), double(0), NULL);
		
		
		
		//temp print
		std::cout << "we in improvepath----------------" << std::endl;
		std::cout << "we are printing open list: " << std::endl;
		printList(open_list);
		std::cout << "we are printing close list: " << std::endl;
		printList(closed_list);
		std::cout << "we are printing incons list: " << std::endl;
		printList(incons_list);
		
		//open_list.push_back(temp);
		int y;
		std::cin >> y;
	}
	
	bool AStarPlanner::ARAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path) {
			//1.g(s_start)=0
			root = new AStarPlannerNode(start, double(0), w*euclidean_distance(goal, start), euclidean_distance(goal, start), NULL);
			//1.g(s_goal)=INF
			goalNode = new AStarPlannerNode(goal, 100000, 100000, double(0), NULL);

			// 2. OPEN=CLOSED+INCONS=Empty
			open_list.clear();
			closed_list.clear();
			incons_list.clear();

			//3 insert s_start into OPEN with fvalue(s_start)
			open_list.push_back(root);

			//4 ImprovePath();
			improvePath();

			
			//5, e'=min(e,g(s_goal)/min_s in OPENuINCONS(g(s)+h(s)))
			int minOPENi = indexWithLeastghARA(open_list);
			//std::cout << " minOPEN: " << minOPENi << std::endl;
			int minINCONSi = indexWithLeastghARA(incons_list);
			//std::cout << " minINCONS: " << minINCONSi << std::endl;
			float minOPEN = 1000000;
			float minINCONS = 1000000;
			if (minOPENi != -1) {
				minOPEN = open_list[minOPENi]->g + open_list[minOPENi]->h;
				//std::cout << " minOPEN: " << minOPEN << std::endl;
			}
			if (minINCONSi != -1) {
				minINCONS = incons_list[minINCONSi]->g + incons_list[minINCONSi]->h;
				//std::cout << " minINCONS: " << minINCONS << std::endl;
			}

			float minOPENuINCONS = 1000000;
			if (minOPEN > minINCONS) {
				minOPENuINCONS = minINCONS;
			}
			else {
				minOPENuINCONS = minOPEN;
			}
			if (w > (goalNode->g)/ minOPENuINCONS) {
				w_s = (goalNode->g) / minOPENuINCONS;
			}
			else {
				w_s = w;
			}
			
			//w_s = minWGs_goal / minOPENuINCONS;
			//public current suboptimal solution
			//6. public suboptimal solution
			agent_path = trace(goalNode);

			std::cout << " w_s: " << w_s << std::endl;
			clockMeasure.updateRealTime();
			//return true;
			std::cout << " time: " << clockMeasure.getCurrentRealTime() << std::endl;
			int x;
			std::cin >> x;
		//7. while w_s>1...
		while (w_s > 1&& clockMeasure.getCurrentRealTime()<maxTime) {
			//8. decrease w.
			w = w_s - .05f;
			std::cout << " w: " << w << std::endl;
			if (w < 0) {
				w = 0;
			}

			//9. move states from INCONS into OPEN
			for (int i = 0; i < incons_list.size(); i++) {
				open_list.push_back(incons_list[i]);
			}
			incons_list.clear();
			//10. nonexistant
			for (int i = 0; i < open_list.size(); i++) {
				open_list[i]->f = open_list[i]->g + w*open_list[i]->h;
			}
			//11.Closed=empty
			closed_list.clear();
			//12.
			improvePath();
			

			//13.
			int minOPENi = indexWithLeastghARA(open_list);
			
			int minINCONSi = indexWithLeastghARA(incons_list);
			
			float minOPEN = 1000000;
			float minINCONS = 1000000;
			if (minOPENi != -1) {
				minOPEN = open_list[minOPENi]->g + open_list[minOPENi]->h;
				
			}
			if (minINCONSi != -1) {
				minINCONS = incons_list[minINCONSi]->g + incons_list[minINCONSi]->h;
				
			}

			float minOPENuINCONS = 0;
			if (minOPEN > minINCONS) {
				minOPENuINCONS = minINCONS;
			}
			else {
				minOPENuINCONS = minOPEN;
			}
			
			if (w > (goalNode->g) / minOPENuINCONS) {
				w_s = (goalNode->g) / minOPENuINCONS;
			}
			else {
				w_s = w;
			}
			//public current suboptimal solution
			
			//14.
			agent_path = trace(goalNode);

			std::cout << " w_s: " << w_s << std::endl;
			
			clockMeasure.updateRealTime();
			std::cout << " time: " << clockMeasure.getCurrentRealTime() << std::endl;
			int x;
			std::cin >> x;
		}

		return true;
		

	}



	//AD* helpers:
	std::vector<float> AStarPlanner::key(AStarPlannerNode *s) {
		std::vector<float> values;
		//1.if(g(s)>rhs(s))
		if (s->g > s->rhs) {
			//2. return [rhs(s)+w*h(s_start,s);rhs(s)]
			values.push_back(s->rhs+w*euclidean_distance(root->point, s->point));
			values.push_back(s->rhs);
		}
		//3. else
		else {
			//4. return [g(s)+h(s_start,s);g(s)]
			values.push_back(s->g + euclidean_distance(root->point, s->point));
			values.push_back(s->g);
		}

		return values;
	}

	void AStarPlanner::updateStateAD(AStarPlannerNode *s) {
		//5.  if(s was not visitited before)
		bool skipOPEN = false;
		bool skipINCONS= false;
		bool skipCLOSED = false;
		int indexOfS=-1;
		for (int j = 0; j < open_list.size(); j++) {
			if (open_list[j]->point == s->point) {
				indexOfS = j;
				skipOPEN = true;
			}
		}
		for (int j = 0; j < incons_list.size(); j++) {
			if (incons_list[j]->point == s->point) {
				skipINCONS = true;
			}
		}
		for (int j = 0; j < closed_list.size(); j++) {
			if (closed_list[j]->point == s->point) {
				skipCLOSED = true;
			}
		}
		if (!skipOPEN && !skipINCONS&&!skipCLOSED) {
			//6. g(s)=inf
			s->g = 10000000;
		}

		//7. if(s is not equal to s_goal)
		if (s->point != goalNode->point) {
			//rhs(s)=min for s' in successors of s (c(s,s')+g(s'))
			std::vector<AStarPlannerNode*> successors = getNeighbors(s);
			// generate q's 8 successors and set their parents to q
			float minimumS_s = 100000000;
			for (int i = 0; i < successors.size(); i++) {
				AStarPlannerNode* s_s = successors[i];
				float costOfMoving = s_s->g + euclidean_distance(s->point, s_s->point);
				if (costOfMoving < minimumS_s) {
					minimumS_s = costOfMoving;
				}
			}
			s->rhs = minimumS_s;
		}
		//8. if(s is in open, remove s from Open)
		if (skipOPEN) {
			open_list.erase(open_list.begin() + indexOfS);
		}


		//9. if(g(s)!=rhs(s))
		if (s->g != s->rhs) {
			//10. if(s!=CLOSED)
			if (!skipCLOSED) {
				//11. Insert s into OPEN with key(s)
				s->key = key(s);
				open_list.push_back(s);
			}
			//12. else
			else {
				//13. insert s into incons
				incons_list.push_back(s);
			}
			
		}
	}

	bool AStarPlanner::KeyAlessthanB(AStarPlannerNode *s, AStarPlannerNode *s2) {
		//std::cout << " 14.----------open_list-------" << std::endl;
		return ((s->key)[0] < (s2->key)[0]) || (((s->key)[0] == (s2->key)[0])&& (s->key)[1] < (s2->key)[1]);
	}

	void AStarPlanner::computeShortestPathAD() {
		//7. while (min of s in OPEN) (key(s))<key(s_star) OR rhs(s_start) not equal to g(s_start)
		//first find that minimum
		//std::cout << "14.----------open_list-------" << std::endl;
		//printList(open_list);
		//std::cout << "14.----------closed_list-------" << std::endl;
		//printList(closed_list);
		//std::cout << "14.----------incons_list-------" << std::endl;
		//printList(incons_list);
		//std::cout << "14.----------------------------" << std::endl;

		//std::vector<float> minkey;
		AStarPlannerNode * minkey;
		int minkeyPosition;
		for (int j = 0; j < open_list.size(); j++) {
			if (j == 0) {
				minkey = open_list[0];
				minkeyPosition = 0;
			}
			if (KeyAlessthanB(open_list[j], minkey)) {
				minkey = open_list[j];
				minkeyPosition = j;
			}
		}
		root->key = key(root);
		//std::cout << "14. root->key[0]: " << root->key[0]<<". root->key[1]"<<root->key[1] << std::endl;
		//std::cout << "14. the s/minkey is: "<< minkey->point << std::endl;
		//std::cout << "(KeyAlessthanB(minkey, root): " << KeyAlessthanB(minkey, root) << std::endl;
		while (KeyAlessthanB(minkey, root)||(root->rhs!=root->g)) {
			//std::cout << "15.----------open_list-------" << std::endl;
			//printList(open_list);
			//std::cout << "15.----------closed_list-------" << std::endl;
			//printList(closed_list);
			//std::cout << "15.----------incons_list-------" << std::endl;
			//printList(incons_list);
			//std::cout << "15.----------------------------" << std::endl;

			//15. remove state s with the minimum key from open
			open_list.erase(open_list.begin() + minkeyPosition);
			//16. if(g(s)>rhs(s))
			if (minkey->g > minkey->rhs) {
				//std::cout << "16.->17." << std::endl;
				//17. g(s)=rhs(s)
				minkey->g = minkey->rhs;
				//18. CLOSED = CLOSEDu{s}
				closed_list.push_back(minkey);
				//19. for all s' which is a predecessor of s, updateState(s')
				std::vector<AStarPlannerNode*> predecessors = getNeighbors(minkey);
				//std::cout << "19. finding predecessorrs" << std::endl;
				for (int i = 0; i < predecessors.size(); i++) {
					// if successor is the goal, stop the search
					//std::cout << "19. updatepredecessor" << std::endl;
					updateStateAD(predecessors[i]);
				}
				//AStarPlannerNode* predecessor = minkey;
				/*while (predecessor->parent != NULL) {
					predecessor = predecessor->parent;
					std::cout << "19. updatepredecessor" << std::endl;
					updateStateAD(predecessor);
				}*/
				//std::cout << "19. no more predecessors" << std::endl;
			}
			//20. else
			else {
				//21. g(s)=inf
				minkey->g = 10000000;
				//22. for all s' which is a predecessor of s and also s, updateState(s')
				updateStateAD(minkey);
				std::vector<AStarPlannerNode*> predecessors = getNeighbors(minkey);
				for (int i = 0; i < predecessors.size(); i++) {
					// if successor is the goal, stop the search
					//std::cout << "22. updatepredecessor" << std::endl;
					updateStateAD(predecessors[i]);
				}
			}
		
			//14. while loop
			//minkey = new AStarPlannerNode;
			for (int j = 0; j < open_list.size(); j++) {
				if (j == 0) {
					minkey = open_list[0];
					minkeyPosition = 0;
				}
				if (KeyAlessthanB(open_list[j], minkey)) {
					minkey = open_list[j];
					minkeyPosition = j;
				}
			}
			minkey->key = key(minkey);
			root->key = key(root);
			//std::cout << "14. root->key[0]: " << root->key[0] << ". root->key[1]" << root->key[1] << std::endl;
			//std::cout << "14. minkey->key[0]: " << minkey->key[0] << ". minkey->key[1]" << minkey->key[1] << std::endl;
			//std::cout << "14. the s/minkey is: " << minkey->point << std::endl;
		}
		//std::cout << "we are done with computorimprovepath" << std::endl;
	}


	bool AStarPlanner::ADStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path) {
		
		if (edgeCostChanges == 0) {
			//pseudocode: http://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf
			//f here is key...
			//1.g(s_start) = INF
			root = new AStarPlannerNode(start, 1000000, 0, 0, NULL);
			//1.rhs(s_start)=INF
			root->rhs = 1000000;
			//1.g(s_goal)=INF
			goalNode = new AStarPlannerNode(goal, 100000, 0, double(0), NULL);
			//2.rhs(s_goal)=0
			goalNode->rhs = 0;
			//2. skip w_0
			//3. OPEN=CLOSED=INCONS=empty
			open_list.clear();
			closed_list.clear();
			incons_list.clear();
			//4. insert s_goal into OPEN with key(s_goal)
			goalNode->key = key(goalNode);
			std::cout << "goalNode->g: " << goalNode->g << std::endl;
			std::cout << "goalNode->rhs: " << goalNode->rhs << std::endl;
			std::cout << "goalNode->h(s_start,goal): " << euclidean_distance(root->point,goalNode->point) << std::endl;
			std::cout << "goalNode->key[0]: " << goalNode->key[0] << ". goalNode->key[1]: " << goalNode->key[1] << std::endl;
			open_list.push_back(goalNode);
			//5.computeorImprovePath()
			computeShortestPathAD();
			//6.publish current w-suboptimal solution
			agent_path = trace(goalNode);
		}
		//7. forever
		while (true) {
			//8. if changes in edge costs are detected
			if (edgeCostChanges == 1) {
				
			}
				

			//12. if significant edge cost changes were observed


			//14. else if w>1
			else if (w > 1) {
				w = w - .5f;
			}

			//16. move states from INCONS into OPEN
			for (int i = 0; i < incons_list.size(); i++) {
				open_list.push_back(incons_list[i]);
			}
			incons_list.clear();
			//17. update the priorities for all s in open according to key(s)
			for (int i = 0; i < open_list.size(); i++) {
				open_list[i]->key = key(open_list[i]);
			}
			//18.CLOSED=empty
			closed_list.clear();
			//19.computeorImprovePath()
			computeShortestPathAD();
			//20. publish current solution
			agent_path = trace(goalNode);
			//21. if w=1
			if (w == 1) {
				//22. wait for changes in edge costs
				return true;
			}
		}


		return true;


	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path) {
		gSpatialDatabase = _gSpatialDatabase;
		int kindOfAStar;
		std::cout << "enter a kind of A* (0:Weighted A*, 1:ARA*, 2:AD*): " << std::endl;
		std::cin >> kindOfAStar;
		std::cout << "enter a weight: " << std::endl;
		std::cin >> w;
		 //w = 10;
		 w_s = 0;
		 maxTime = 100000;
		 edgeCostChanges = 0;
		 clockMeasure.reset();
		 clockMeasure.updateRealTime();
		 //clockMeasure.advanceSimulationAndUpdateRealTime();
		 
		 switch (kindOfAStar) {
			
			case 1:
				return ARAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
			case 2:
				//first assessment
				ADStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
				//change the path non-drastically
				//edgeCostChanges = 1;
				//SpatialDatabaseItemPtr object;
				//gSpatialDatabase->addObject(object, AxisAlignedBox(4, 8, 0, 1, 0, 2));
					
				//ADStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);


				//change the path cost drastically
				//edgeCostChanges = 2;
				//ADStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
				return true;
				//return ADStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
			default:
				return weightedAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
		 }
		 
		//return ARAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
		//return ADStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);

	}

}

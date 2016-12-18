#include "obstacles/GJK_EPA.h"


//GJK finish
//get support of points with certain direction
//input: list of vertices, count and dirtection
Util::Vector SteerLib::GJK_EPA::getSupport(std::vector<Util::Vector> vertices, int count, Util::Vector d) {
    float highest = -FLT_MAX;
    Util::Vector support(0, 0, 0);

    for (int i = 0; i < count; ++i) {
        Util::Vector v = vertices[i];
        float dot = v.x*d.x + v.z*d.z;
        if (dot > highest) {
            highest = dot;
            support = v;
        }
    }

    return support;
}

//figure out the new simplex given old simplex and direction
bool SteerLib::GJK_EPA::doSimplex(std::vector<Util::Vector> &list, Util::Vector &d) {
    //either line or triangle

    //line case
    if (list.size() == 2) {

        //AB->
        Util::Vector pointA = list[1];
        Util::Vector pointB = list[0];
        Util::Vector vectorAB = pointB - pointA;

        Util::Vector vectorAO = -pointA;

        //if AB.A0>0
        if (vectorAB*vectorAO > 0) {
            list.clear();
            //then simplex is A,B
            list.push_back(pointB);
            list.push_back(pointA);

            //d=ABxA0xAB
            d = cross(cross(vectorAB, vectorAO), vectorAB);
            return false;
        }
        else {
            //then simplex is A
            list.clear();
            list.push_back(pointA);
            //d=AO
            d = vectorAO;
            return false;
        }
    }
    //triangle case

    else {
        Util::Vector pointA = list[2];
        Util::Vector pointB = list[1];
        Util::Vector pointC = list[0];
        Util::Vector vectorAB = pointB - pointA;
        Util::Vector vectorAC = pointC - pointA;
        Util::Vector vectorABC = cross(vectorAB,vectorAC);
        Util::Vector vectorAO = -pointA;

        if (cross(vectorABC, vectorAC)*vectorAO > 0) {
            if (vectorAC*vectorAO > 0) {
                //simplex=[A,C]
                //direction=ACxAoxAC
                list.clear();
                list.push_back(pointC);
                list.push_back(pointA);

                d = cross(cross(vectorAC, vectorAO), vectorAC);
                return false;
            }
            else {
                if (vectorAB*vectorAO > 0) {
                    //simplex=[A,B]
                    //direction=ABxAoxAB
                    list.clear();
                    list.push_back(pointB);
                    list.push_back(pointA);

                    d = cross(cross(vectorAB, vectorAO), vectorAB);
                    return false;
                }
                else {
                    //simplex=[A]
                    //direction=AO
                    list.clear();
                    list.push_back(pointA);
                    d = vectorAO;
                    return false;
                }
            }
        }
        else {
            if (cross(vectorAB, vectorABC)*vectorAO > 0) {
                if (vectorAB*vectorAO > 0) {
                    //simplex=[A,B]
                    //direction=ABxAoxAB
                    list.clear();
                    list.push_back(pointB);
                    list.push_back(pointA);

                    d = cross(cross(vectorAB, vectorAO), vectorAB);
                    return false;
                }
                else {
                    //simplex=[A]
                    //direction=AO
                    list.clear();
                    list.push_back(pointA);
                    d = vectorAO;
                    return false;
                }
            }
            else {
                //else its in triangle and we are done
                return true;
            }
        }


    }

    return true;

}


//RETURNS: true if collides,SIMPLEX
// false if doesnt collide, null
bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex) {
    /*for (int i = 0; i < _shapeA.size(); i++) {
        std::cout << "_shapeA at " << i << ":" << _shapeA[i] << std::endl;
    }
    for (int i = 0; i < _shapeB.size(); i++) {
        std::cout << "_shapeB at " << i << ":" << _shapeB[i] << std::endl;
    }*/


    //get some d: (1,0,1) for example
    Util::Vector d(1, 0, 1);
    //S=Support(A,D)-Support(B,-D)
    Util::Vector point = getSupport(_shapeA, _shapeA.size(), d) - getSupport(_shapeB, _shapeB.size(), -d);

    //[]=S
    std::vector<Util::Vector> list;
    list.push_back(point);


    //D=-S
    Util::Vector direction = -point;//(-point.x, -point.y, -point.z);


    while (true) {


        Util::Vector pointA = getSupport(_shapeA, _shapeA.size(), direction) - getSupport(_shapeB, _shapeB.size(), -direction);
        //if dot product between pointA and direction <0

        //int n;
        //std::cin >> n;
        if (pointA*direction < 0) {
            simplex.clear();
            return false;
        }
        list.push_back(pointA);
        if (doSimplex(list, direction)) {
            simplex = list;
            return true;
        }
    }
}

//finds the neerest edge to the origin in polytope
int findNeerestEdgeToOrigin(std::vector<Util::Vector> polytope, float &distanceToEdge) {
	//first set distance to max float number
	float distance = std::numeric_limits<float>::max();
    int index = 0;

	//go through each edge in polytope
    for (int i = 0; i < polytope.size(); i++) {

        int j;
		//set second point to either i+1 or 0 depending on whether or not i=polytope.size()
        if (i + 1 == polytope.size()) {
            j = 0;
        }
        else {
            j = i + 1;
        }

        Util::Vector a = polytope[i];
        Util::Vector b = polytope[j];
        
		//edge
		Util::Vector c = b-a;

		//find normal to edhe
        Util::Vector n = normalize(cross((cross(c, a)), c));
        
		//find distance from edge to origin
		float distanceToOrigin = n*a;

		//find minimum distance to origin
        if (distanceToOrigin < distance) {
			//set minimum distance to origin to varible by reference
            distanceToEdge = distanceToOrigin;

			//set to minimum
            distance = distanceToOrigin;

			//set index to index of first point on edge closest to origin
            index = i;
        }
    }
    return index;
}

Util::Vector SteerLib::GJK_EPA::findPenetrationVector(std::vector<Util::Vector> A, std::vector<Util::Vector> B, std::vector<Util::Vector> simplex) {


    float distanceToEdge=0;

	//go through 
    while (true) {

        //find index of first point in closest edge to origin
        int index = findNeerestEdgeToOrigin(simplex,distanceToEdge);

        //find index of second point in nearest edge
        int index2 = index + 1;
        if (index2 == simplex.size()) {
            index2 = 0;
        }
        
        //nearest edge to origin
        Util::Vector nearestEdge = simplex[index] - simplex[index2];

        //normal of nearestEdge:
        Util::Vector nearestEdgeNormal = cross(Util::Vector(0, 1, 0), nearestEdge);

		//normalized normal of nearestEdge
        Util::Vector normalizedNearestEdgeNormal = normalize(nearestEdgeNormal);

        //get support point
        Util::Vector point = getSupport(A, A.size(), nearestEdgeNormal) - getSupport(B, B.size(), -nearestEdgeNormal);

		//get distance from origin to point
        float distancePoint = point*normalizedNearestEdgeNormal;

		//if distancePoint -distanceToEdge < certain threshold
		if (distancePoint -distanceToEdge< 0.0001) {
			//we are done with the EPA and just need to compute the normal, the penetration vector
            Util::Vector normal = cross(nearestEdge, Util::Vector(0, 1, 0));
            return (normal / normal.length()) * distanceToEdge;
        } else {
            //keep expanding the polytope
            simplex.insert(simplex.begin()+index+1, point);
        }
    }


}



//RETURNS: true if collision
//ARGUMENTS: RETURN: return_penetration_depth: penetration depth claculated by EPA if collision
//			 RETURN: return_penetration_vector: penetration vector calculated by EPA if collision
//			 INPUT: _shapeA: shape A. array of vector points: which have x, y, z components
//			 INPUT: _shapeB: shape B. array of vector points: which have x, y, z components
//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
    std::vector<Util::Vector> simplex;
    float isColliding = GJK(_shapeA, _shapeB, simplex);
    if (isColliding == true) {
        return_penetration_vector = findPenetrationVector(_shapeA, _shapeB, simplex);
        return_penetration_depth = return_penetration_vector.length();
        return_penetration_vector = normalize(return_penetration_vector);
        return true;
    }
    else {
        return_penetration_depth = 0;
        return_penetration_vector.zero();
        return false;
    }
}

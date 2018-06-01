#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <cmath>
#include <iostream>
#include <queue>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <math.h>
#include <pthread.h>
#include <eigen3/Eigen/Dense>
#include <bits/stdc++.h>

using namespace std;
using namespace cv;

#define PI 3.141597

//Parameters
int max_iter = 1000;
double alpha = 0.1,beta = 1;
double kmax,rs;
double epsilon,muX;
double dt = 0.0;
double t_max = 1.0; //time until which rewiring and expanding is to be done

/* Utils */
template <typename T>
class DynamicArray
{
public:
    DynamicArray(){};

    DynamicArray(int rows, int cols): dArray(rows, vector<T>(cols)){}

    vector<T> & operator[](int i) 
    { 
      return dArray[i];
    }
    const vector<T> & operator[] (int i) const 
    { 
      return dArray[i];
    }
    void resize(int rows, int cols)//resize the two dimentional array .


    {
        dArray.resize(rows);
        for(int i = 0;i < rows;++i) dArray[i].resize(cols);
    }
private:
    vector<vector<T> > dArray;  
};

//Util data structures
struct Node_ {
int x,y;
double cost;
};

//Util variables
Mat img;
Node_ start,goal;
vector<Node_> nodes;
bool isPathToGoal = false;
queue<Node_> Qr,Qs;
vector<Node_> Qs_vector;
vector<Node_> Qr_vector;
DynamicArray<Point> parent;

/*-----------Util functions------------*/
double getDis(Node_ a,Node_ b) {
	return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
}

int checkValid(int x,int y) {
	return (y >= 0 && y < img.rows && x >= 0 && x < img.cols);
}

int checkForLine(Node_ curN,Node_ newN) {
	//y = mx (-mx1 + y1)
	//cout << "A";
	//handle the m = infinity case
	double m;
	if(curN.x == newN.x) m = 1e6;
	else
	m = (((newN.y-curN.y)*1.0)/(newN.x-curN.x));
	double c = (-m*curN.x) + curN.y;
	//cout << "y = " << m << "x + " << c << endl;
	
	double x;
	for(x = curN.x;x <= newN.x;x++) {
		//cout << "---------------" << endl;
		int y = (m*x + c);
		//cout << x << " " << y << " " << (int)(gs.at<uchar>(y,x)) << endl;
		if(img.at<uchar>(y,x) == 255)  {
			//cout << "True" << endl;
			return 0;
			break;
		}
	}

	return 1;

}
/*---------End of Util Functions--------*/



/* ******* RRT* Utils Functions ********* */

//finds closest node in tree to given node
Node_ findClosest(Node_ node) {
	double minDis = FLT_MAX;
	Node_ nearest;
	for(int i = 0;i < nodes.size();i++) {
		if(getDis(node,nodes[i]) < minDis) {
			nearest.x = nodes[i].x;
			nearest.y = nodes[i].y;
		}
	}

	return nearest;
}

Node_ findClosestInVector(Node_ node,vector<Node_> nodes_) {
	double minDis = FLT_MAX;
	Node_ nearest;
	for(int i = 0;i < nodes_.size();i++) {
		if(getDis(node,nodes_[i]) < minDis) {
			nearest.x = nodes_[i].x;
			nearest.y = nodes_[i].y;
		}
	}

	return nearest;
}

vector<Node_> findNeighbours(Node_ node) {
	epsilon = sqrt((muX*kmax)/(PI*nodes.size()));

	vector< Node_> neighbours;
	for(int i = 0;i < nodes.size();i++) {
		if(getDis(node,nodes[i]) < epsilon) {
			neighbours.push_back(nodes[i]);
		}
	}

	return neighbours;
}

vector<Node_> findNeighboursToVector(vector<Node_> nodes_,Node_ node) {
	epsilon = sqrt((muX*kmax)/(PI*nodes.size()));

	vector< Node_> neighbours;
	for(int i = 0;i < nodes_.size();i++) {
		if(getDis(node,nodes_[i]) < epsilon) {
			neighbours.push_back(nodes_[i]);
		}
	}

	return neighbours;
}

bool checkNodeInQs(Node_ x_s) {
	queue<Node_> q_temp;

	bool out = false;

	while(!(Qs.empty())) {
		Node_ tem = Qs.front();
		q_temp.push(tem);

		if(tem.x == x_s.x && tem.y == x_s.y) 
			out = true;
		
		Qs.pop();
	}

	while(!(q_temp.empty())) {
		Qs.push(q_temp.front());
		q_temp.pop();
	}

	return out;
}

/* * * * * * * * * * * * * * * * * * * * * * * * */

Node_ randomAlongLine(Node_ nearest) { //here this is the nearest node in the tree to the goal
	int min_x,max_x,min_y,max_y;

	if(goal.x > nearest.x) {
		min_x = nearest.x;
		max_x = goal.x;
		min_y = nearest.y;
		max_y = goal.y;
	}

	else {
		min_x = goal.x;
		max_x = nearest.x;
		min_y = goal.y;
		max_y = nearest.y;
	}

	//sample randomly in the min-max line
	int x_rand = drand48()*max_x;
	if(x_rand < min_x) x_rand+=min_x;

	//eqn of min-max line
	double m = (1.0*(max_y - min_y))/(max_x - min_x);
	double c = (max_y - (m*min_y));

	int y_rand = (int)(m*x_rand + c);

	Node_ rnd_node;
	rnd_node.x = x_rand;
	rnd_node.y = y_rand;

	return rnd_node;
}

Node_ randomSample() {
	float Pr = drand48();
	Node_ rnd_node;

	//cases
	if(Pr > 1.0 - alpha) {
		//sapmle along line
		Node_ nearest = findClosest(goal);
		rnd_node = randomAlongLine(nearest);
	}

	else if(Pr <= ((1.0 - alpha)/beta) && (isPathToGoal == true)) {
		rnd_node.x = drand48()*img.cols;
		rnd_node.y = drand48()*img.rows;
	}

	else {
		//Ellipsis random sampling
		/*Temporary*/
		rnd_node.x = drand48()*img.cols;
		rnd_node.y = drand48()*img.rows;
		/* * * * * */
		//...to be done
	}

	return rnd_node;

}

vector<Node_> getSpatialIndexedNodes(Node_ node) {
	vector<Node_> Xsi;

	//not using grid squares for now :: TODO --> use grid squares
	for(int i = 0;i < nodes.size();i++) {
		/*Temp*/
		Xsi.push_back(nodes[i]);


		//add code later
	}

	return Xsi;
}

//----------Add node to tree------------ :: Done
void addNodeToTree(Node_ x_new,Node_ x_closest,vector<Node_> neighbours) {
	Node_ x_min;
	double c_min;

	x_min.x = x_closest.x;
	x_min.y = x_closest.y;
	x_min.cost = x_closest.cost;

	c_min = x_closest.cost + getDis(x_closest,x_new);

	for(int i = 0;i < neighbours.size();i++) {
		double c_new = neighbours[i].cost + getDis(neighbours[i],x_new);

		if(c_new < c_min && checkForLine(neighbours[i],x_new)) {
			c_min = c_new;
			x_min.x = neighbours[i].x;
			x_min.y = neighbours[i].y;
			x_min.cost = neighbours[i].cost;
		}
	}

	nodes.push_back(x_new);
	parent[x_new.x][x_new.y] = Point(x_min.x,x_min.y);
}


/*------Rewiring---------*/

//-------------------------------: Done
void rewireRandomNodes(Node_ rnd_node) {
	while(dt <= t_max || (!Qr.empty())) {
		Node_ xr = Qr.front();
		Qr.pop();

		vector<Node_> Xsi = getSpatialIndexedNodes(rnd_node);
		vector<Node_> neighbours = findNeighboursToVector(Xsi,xr);

		for(int i = 0;i < neighbours.size();i++) {
			double c_old = neighbours[i].cost;
			double c_new = xr.cost + getDis(xr,neighbours[i]);

			if(c_new < c_old && checkForLine(xr,neighbours[i])) {
				parent[neighbours[i].x][neighbours[i].y] = Point(xr.x,xr.y);
				Qr.push(neighbours[i]);
			}
		}
	}
}

//-----------------------: Done
void rewireFromRoot(Node_ rnd_node) {
	if(Qs.empty()) {
		Qs.push(start);
		// Qs_vector.push_back(start);
	}

	while(dt <= t_max || (!Qs.empty())) {
		Node_ xs = Qs.front();
		Qs.pop();

		vector<Node_> Xsi = getSpatialIndexedNodes(rnd_node);
		vector<Node_> neighbours = findNeighboursToVector(Xsi,xs);

		for(int i = 0;i < neighbours.size();i++) {
			double c_old = neighbours[i].cost;
			double c_new = xs.cost + getDis(xs,neighbours[i]);

			if(c_new < c_old && checkForLine(xs,neighbours[i])) {
				parent[neighbours[i].x][neighbours[i].y] = Point(xs.x,xs.y);
			}
		

		if(!checkNodeInQs(xs)) {
			Qs.push(neighbours[i]);
		}
	}
	}
}
/*-----------------------*/

//-----------Expansion---------------- : Done
void Expand() {
	Node_ rnd_node = randomSample();
	vector<Node_> Xsi = getSpatialIndexedNodes(rnd_node);
	Node_ x_closest = findClosestInVector(rnd_node,Xsi);

	if(checkForLine(x_closest,rnd_node)) {
		vector<Node_> neighbours = findNeighboursToVector(Xsi,rnd_node);

		if(neighbours.size() < kmax || getDis(x_closest,rnd_node) > rs) {
			//TODO : ADD NODE TO TREE

			Qr.push(rnd_node);
		}

		else {
			Qr.push(x_closest);
		}

		//TODO : Rewire random node
		rewireRandomNodes(rnd_node);

	}

	//TODO : Rewire from root
	rewireFromRoot(rnd_node);
}

int main() {
	img = imread("map.png");
	parent = DynamicArray<Point>(img.cols,img.rows);

	for(int i = 0;i < max_iter;i++) {

	}

	return 0;
}
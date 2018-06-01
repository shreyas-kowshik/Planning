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

using namespace cv;
using namespace std;
using namespace Eigen;

Mat img,tree;
const int step_size = 10,max_iter = 6000;
int r = 20;
const double radius_factor = 25.0;

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

struct Node_ {
int x,y;
double cost;
Node_ *parent;
};

vector<Node_> nodes;
vector<vector<double> > costArr;
DynamicArray<Point> parent;
Node_ start,goal;

/*---End Utils---*/

int checkValid(int x,int y) {
	return (y >= 0 && y < img.rows && x >= 0 && x < img.cols && img.at<uchar>(y,x) < 200);
}

int checkForLine(Node_ curN,Node_ newN) {
	//y = mx (-mx1 + y1)
	//handle the m = infinity case
	double m;
	if(curN.x == newN.x) m = 1e6;
	else
	m = (((newN.y-curN.y)*1.0)/(newN.x-curN.x));
	double c = (-m*curN.x) + curN.y;
	
	double x;
	for(x = curN.x;x <= newN.x;x++) {
		int y = (m*x + c);

		if(img.at<uchar>(y,x) == 255)  {
			return 0;
			break;
		}
	}

	return 1;

}

double getDis(Node_ a,Node_ b) {
	return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
}

Node_ getRandomNode_() {
	int y = drand48()*img.rows;
	int x = drand48()*img.cols;

	Node_ node;

	if(checkValid(x,y)) {
		node.x = x;node.y = y; //here it's cost dosen't matter,we use it only for point sampling
		return node;
	}

	getRandomNode_();
}

Node_ findClosest(Node_ newNode_) {
	float min = numeric_limits<float>::infinity();
	Node_ nearest;

	for(int i = 0;i<nodes.size();i++) {
		float dis = pow(newNode_.x - nodes[i].x,2) + pow(newNode_.y - nodes[i].y,2);
		if(dis < min) {
			min = dis;
			nearest.x = nodes[i].x;
			nearest.y = nodes[i].y;
			nearest.cost = nodes[i].cost;
		}
	}

	return nearest;
}

Node_ update(Node_ cur,Node_ newNode_) {
	//convert to vector
	Vector2f cur_v = Vector2f(cur.x,cur.y);
	Vector2f newNode__v(newNode_.x,newNode_.y);

	Vector2f dif = newNode__v - cur_v;
	Vector2f dir = dif/(dif.norm());

	Vector2f ret_v;
	if(getDis(cur,newNode_) >= step_size)
		ret_v = cur_v + step_size*dir;
	else
		ret_v = cur_v + getDis(cur,newNode_)*dir;

	Node_ ret;
	ret.x = ret_v[0];
	ret.y = ret_v[1];

	return ret;
}

vector<Node_> findNeighbours(Node_ node) {
	//optimal r
	r = radius_factor * step_size * sqrt(1.0*(log(nodes.size())/nodes.size()));

	vector< Node_> neighbours;
	for(int i = 0;i < nodes.size();i++) {
		if(getDis(node,nodes[i]) < r) {
			neighbours.push_back(nodes[i]);
		}
	}

	return neighbours;
}

Node_ getNearestNeighbour(vector<Node_> &neighbours,Node_ node) {
	double min = FLT_MAX;
	Node_ nearestNeighbour;

	for(int i = 0;i < neighbours.size();i++) {
		if((getDis(neighbours[i],node) + neighbours[i].cost) < min && checkForLine(neighbours[i],node)) {
			min = getDis(neighbours[i],node) + neighbours[i].cost;
			nearestNeighbour.x = neighbours[i].x;
			nearestNeighbour.y = neighbours[i].y;
			nearestNeighbour.cost = neighbours[i].cost;
		}
	}

	return nearestNeighbour;
}

bool checkIfNearestInNeighbours(vector<Node_> &neighbours,Node_ nearest) {
	bool out = false;
	for(int i = 0;i < neighbours.size();i++) {
		if(neighbours[i].x == nearest.x && neighbours[i].y  == nearest.y) {
			out = true;
			break;
		}
	}

	return out;
}

void RRT_STAR(Node_ start,Node_ goal) {
	bool reached = false;
	Mat tem = img.clone();
	
	//clear any previous instances of callbacks
	nodes.clear();
	start.cost = 0;
	nodes.push_back(start);

	for(int iter = 0;iter < max_iter;iter++) {
		//check convergence
		for(int i = 0;i < nodes.size();i++) {
			if(getDis(nodes[i],goal) < step_size) {
				parent[goal.x][goal.y] = Point(nodes[i].x,nodes[i].y);
				nodes.push_back(goal);
				reached = true;
				break;
			}
		}

		if(reached) break;

		Node_ newNode_T = getRandomNode_();
		Node_ nearestNode_ = findClosest(newNode_T);

		Node_ newNode_ = update(nearestNode_,newNode_T);

		if(checkValid(newNode_.x,newNode_.y) && checkForLine(nearestNode_,newNode_)) {
			//steering
			parent[newNode_.x][newNode_.y] = Point(nearestNode_.x,nearestNode_.y);
			newNode_.cost = nearestNode_.cost + getDis(newNode_,nearestNode_);
			//steering done

			vector<Node_> neighbours = findNeighbours(newNode_);

			if(checkIfNearestInNeighbours(neighbours,nearestNode_) == false)
				neighbours.push_back(nearestNode_);

			if(neighbours.size() > 0) {

				Node_ nearestNeighbour = getNearestNeighbour(neighbours,newNode_);

				if(!checkForLine(nearestNeighbour,newNode_)) continue;
				parent[newNode_.x][newNode_.y] = Point(nearestNeighbour.x,nearestNeighbour.y);
				newNode_.cost = nearestNeighbour.cost + getDis(newNode_,nearestNeighbour);
			
				//rewire
				for(int i = 0;i < neighbours.size();i++) {
					if(((newNode_.cost + getDis(newNode_,neighbours[i])) < neighbours[i].cost) && checkForLine(newNode_,neighbours[i])) {
						neighbours[i].cost = newNode_.cost + getDis(newNode_,neighbours[i]);
						parent[neighbours[i].x][neighbours[i].y] = Point(newNode_.x,newNode_.y);
					}
				}

			}

			nodes.push_back(newNode_);
			// tem.at<uchar>(newNode_.y,newNode_.x) = 255;
			line(tem,Point(parent[newNode_.x][newNode_.y].x,parent[newNode_.x][newNode_.y].y),Point(newNode_.x,newNode_.y),Scalar(128), 2, CV_AA);
		}

		// imshow("DISPLAY",tem);
		// waitKey(1);
	}

	if(reached == false) {
		Node_ nearestNode_ = findClosest(goal);
		parent[goal.x][goal.y] = Point(nearestNode_.x,nearestNode_.y);
		nodes.push_back(goal);
	}

	tree = tem.clone();

}

void show(Node_ start,Node_ goal) {
	Mat display = img.clone();

	for(int i = 0;i < nodes.size();i++) {
		cout << nodes[i].x << "," << nodes[i].y << " :: " << nodes[i].cost << endl;
	}

	//show path
	Mat path = img.clone();
	Node_ goal_;
	goal_.x = nodes[nodes.size()-1].x;
	goal_.y = nodes[nodes.size()-1].y;

	while(goal_.x != start.x && goal_.y != start.y) {
		cout << goal_.x << "," << goal_.y << endl;
		tree.at<uchar>(goal_.y,goal_.x) = 128;

		line(tree,Point(goal_.x,goal_.y),parent[goal_.x][goal_.y], Scalar(255), 2, CV_AA);

		imshow("PATH",tree);
		waitKey(10);

		double x = (parent[goal_.x][goal_.y]).x;
		double y = (parent[goal_.x][goal_.y]).y;
		goal_.x =  x;
		goal_.y =  y;
	}

	imshow("Path",tree);
	// imshow("A",display);
	waitKey(0);
	imwrite("Output.png",tree);
}

int main() {	
	img = imread("map.png",0);
	parent = DynamicArray<Point>(img.cols,img.rows);

	start.x = 0;
	start.y = 0;
	goal.x = img.cols - 1;
	goal.y = img.rows - 1;

	circle(img,Point(start.x,start.y),5,Scalar(128),2,8,0);
	circle(img,Point(goal.x,goal.y),5,Scalar(128),2,8,0);;

	if(img.at<uchar>(goal.y,goal.x) > 200)
		cout << "Invalid goal\n";
	RRT_STAR(start,goal);
	cout << "A" << endl;
	show(start,goal);
	return 0;
}

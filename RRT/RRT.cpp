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

//forward declarations
void show(Vector2f start,Vector2f goal);

Mat img,gs;
const int step_size = 10,max_iter = 500;

vector<Vector2f> nodes;

/*** Util Data Structures ***/
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

DynamicArray<int> myMap;
DynamicArray<Vector2f> parent;

int checkValid(int x,int y) {
	return (y >= 0 && y < img.rows && x >= 0 && x < img.cols && myMap[x][y] > 0);
}

double getDis(Vector2f a,Vector2f b) {
	return sqrt(pow(a[0]-b[0],2) + pow(a[1]-b[1],2));
}

int checkForLine(Vector2f curN,Vector2f newN) {
	//y = mx (-mx1 + y1)
	//cout << "A";
	//handle the m = infinity case
	double m;
	if(curN[0] == newN[0]) m = 1e6;
	else
	m = (((newN[1]-curN[1])*1.0)/(newN[0]-curN[0]));
	double c = (-m*curN[0]) + curN[1];
	//cout << "y = " << m << "x + " << c << endl;
	
	double x;
	for(x = curN[0];x <= newN[0];x++) {
		//cout << "---------------" << endl;
		int y = (m*x + c);
		//cout << x << " " << y << " " << (int)(gs.at<uchar>(y,x)) << endl;
		if(myMap[x][y] == 0)  {
			//cout << "True" << endl;
			return 0;
			break;
		}
	}

	return 1;

}

Vector2f getRandomNode() {
	int y = drand48()*img.rows;
	int x = drand48()*img.cols;
	// cout << "random values: " << x << " " << y << " " << endl;
	Vector2f node;

	if(checkValid(x,y)) {
		node = Vector2f(x,y);
		return node;
	}
	//cout << endl << "Random Node out of bounds" << endl;
	getRandomNode();
}

Vector2f findClosest(Vector2f newNode) {
	float min = numeric_limits<float>::infinity();
	Vector2f nearest;

	for(int i = 0;i<nodes.size();i++) {
		float dis = pow(newNode[0] - nodes[i][0],2) + pow(newNode[1] - nodes[i][1],2);
		if(dis < min) {
			min = dis;
			nearest = nodes[i];
		}
	}

	return nearest;
}

Vector2f update(Vector2f cur,Vector2f newNode) { {}
	Vector2f dif = newNode - cur;
	Vector2f dir = dif/(dif.norm());
	Vector2f ret = cur + step_size*dir;

	ret[0] = (int)(ret[0]);
	ret[1] = (int)(ret[1]);

	//cout <<  "Update : " << cur[0] << " " << dir[0] << endl;
	//ret = wrapInt(ret);

	return ret;
}

void RRT(Vector2f start,Vector2f goal) {
	bool reached = false;

	nodes.push_back(start);

	for(int i = 0;i<max_iter;i++) {
		// cout << i << endl;

		//check for convergence
		for(int i = 0;i < nodes.size();i++) {
			cout << getDis(nodes[i],goal) << endl;
			if(getDis(nodes[i],goal) <= step_size) {
				parent[goal[0]][goal[1]] = Vector2f(nodes[i][0],nodes[i][1]);
				nodes.push_back(goal);
				reached = true;
				break;
			}
		}
		if(reached) break;
		//convergence check done

		Vector2f newNodeT = getRandomNode();
		Vector2f current = findClosest(newNodeT);
		Vector2f newNode;
		newNode = update(current,newNodeT);

		while((!checkValid(newNode[0],newNode[1])) || (!(checkForLine(current,newNode)))) {
			// cout << newNode[0] << " " << newNode[1] << endl;
			newNodeT = getRandomNode();
			current = findClosest(newNodeT);
			newNode = update(current,newNodeT);
		}

		if(getDis(current,newNodeT) < step_size) {
			newNode = newNodeT;
		}

		parent[newNode[0]][newNode[1]] = Vector2f(current[0],current[1]);
		nodes.push_back(newNode);

		line(img,Point(current[0],current[1]),Point(newNode[0],newNode[1]),Scalar(255,0,0), 2, CV_AA);

		imshow("output",img);
		waitKey(1);

	}

	//base case for final output
	if(!reached) {
		parent[goal[0]][goal[1]] = findClosest(goal);
		nodes.push_back(goal);
	}

	show(start,goal);
}

void show(Vector2f start,Vector2f goal) {	
	Vector2f current;
	current[0] = goal[0];
	current[1] = goal[1];

	cout << "Show Started" << endl;

	while((current[0] != start[0]) && (current[1] != start[1])) {
		cout << current[0] << " " << current[1] << endl;
		cout << parent[current[0]][current[1]][0] << " " << parent[current[0]][current[1]][1] << endl;
		line(img,Point(current[0],current[1]),Point(parent[current[0]][current[1]][0],parent[current[0]][current[1]][1]),Scalar(0,255,0), 2, CV_AA);
		current = parent[current[0]][current[1]];
		imshow("A",img);
		waitKey(10);
	}
}		

int main() {
	img = imread("map.png",1);
	myMap = DynamicArray<int>(img.cols,img.rows);
	gs = imread("map.png",0);
	parent = DynamicArray<Vector2f>(img.cols,img.rows);

	// cout << img.cols << " " << img.rows << endl;

	//create the map
	for(int i = 0;i<img.cols;i++) {
		for(int j = 0;j<img.rows;j++) {
			if(gs.at<uchar>(j,i) > 200) myMap[i][j] = 0;
			else myMap[i][j] = 1;
		}
	}

	namedWindow("output",WINDOW_NORMAL);

	Vector2f start(0,0);
	Vector2f goal(img.cols-1,img.rows-1);

	cout << "RRT about to start\n";
	RRT(start,goal);

	imshow("output",img);
	waitKey(0);
	imwrite("Output.png",img);

	return 0;
}


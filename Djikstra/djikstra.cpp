#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <bits/stdc++.h>
#include <iostream>

using namespace std;
using namespace cv;

class Node_ {
public:
	Point p;
	double dist;

	Node_(int x,int y,double dis) {
		this->p = Point(x,y);
		this->dist = dis;
	}
};

// class Compare {
// public:
// 	bool operator()(Node_ a,Node_ b) {
// 		return a.dist > b.dist;
// 	}
// };

bool operator<(const Node_& a,const Node_& b) {
	return a.dist > b.dist;
}

bool isValid(int x,int y,Mat img) {
	if(x < 0 | y < 0 | x >= img.cols | y >= img.rows) return false;
	if(img.at<uchar>(y,x) == 255) return false;
	return true;
}

void plan(Mat img,Point source,Point goal) {
	bool reached = false;
	Mat show_expanding = img.clone();
	Mat obstacles = img.clone();

	priority_queue<Node_> Q;
	vector<vector<int> > visited(img.cols,vector<int>(img.rows,0));
	vector<vector<Node_> > parent(img.cols,vector<Node_>(img.rows,Node_(source.x,source.y,0)));
	vector<vector<double> > distance(img.cols,vector<double>(img.rows,FLT_MAX));

	cout << "Node_s pushed\n";

	Node_ src(source.x,source.y,0);
	Q.push(src); 
	distance[source.x][source.y] = 0;
	parent[source.x][source.y] = Node_(source.x,source.y,0);

	while(!Q.empty()) {
		Node_ current = Q.top();
		Q.pop();
		visited[current.p.x][current.p.y] = 1;

		//iterate neighbours of current Node_
		for(int k = -1;k <= 1;k++) {
			for(int l = -1;l <= 1;l++) {
				if(k == 0 & l == 0) continue; //if we get the current point
				if(isValid(current.p.x + k,current.p.y + l,obstacles)) {
					// if(visited[current.p.x + k][current.p.y + l] == 0) {
						double wt = 1.0;
						if((fabs(k) + fabs(l)) != 1) wt = 1.414;
						if(distance[current.p.x][current.p.y] + wt < distance[current.p.x + k][current.p.y + l]) {
							show_expanding.at<uchar>(current.p.y,current.p.x) = 255;
							distance[current.p.x + k][current.p.y + l] = distance[current.p.x][current.p.y] + wt;
							parent[current.p.x + k][current.p.y + l] = Node_(current.p.x,current.p.y,0); //distance here dose'nt matter

							if(((current.p.x + k) == goal.x) && ((current.p.y + l) == goal.y)) {
								reached = true;
								break;
							}

							Q.push(Node_(current.p.x + k,current.p.y + l,distance[current.p.x][current.p.y] + wt));
				}
			// }
			}
			// imshow("expansion",show_expanding);
			// waitKey(1);
			}
			if(reached)
				break;
		} //end of relaxation
		if(reached)
			break;

	} //end of djikstra's loop

	cout << "End of loop\n";

	cout << "---------Showing Path--------\n";

	Node_ current = Node_(goal.x,goal.y,0);
	Node_ cur_next = Node_(goal.x,goal.y,0);
	int x_cur = current.p.x,y_cur = current.p.y;

	//showing path
	while(1) {
		if(x_cur == source.x & y_cur == source.y) break;
		//cout << "ANDAR" << endl;
		current = Node_(cur_next.p.x,cur_next.p.y,0);
		img.at<uchar>(current.p.y,current.p.x) = 255;
		// cout << x_cur << "," << y_cur << endl;
		cur_next = Node_(parent[current.p.x][current.p.y].p.x,parent[current.p.x][current.p.y].p.y,0);
		x_cur = cur_next.p.x;
		y_cur = cur_next.p.y;
		// imshow("DJI",img);
		// waitKey(10);
	}

	cout << "COST : " << distance[goal.x][goal.y] << endl;
	namedWindow("DJI",WINDOW_AUTOSIZE);
	imshow("DJI",img);
	// imshow("EXPANDED",show_expanding);
	waitKey(0);
	imwrite("Output.png",img);
}

int main() {
	clock_t t;
	t = clock();
	Mat img = imread("chinti.png",0);
	// Mat img(1000,1000,CV_8UC1,Scalar(0));
	Point source(300,450);
	Point goal(300,100);

	if(isValid(goal.x,goal.y,img))
		plan(img,source,goal);
	else
		cout << "Invalid goal" << endl;

	t = clock() - t;
	cout << "Time : " << t << endl;
	cout << "BATA" << endl;
}

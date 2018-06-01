#ifndef C_TEST_H
#define C_TEST_H

#include<bits/stdc++.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
using namespace cv;
using namespace std;

extern vector<Mat> c_spaces;
extern Mat grid[360];
Mat img;
vector<Vec4i> hierarchy;
vector<vector<Point> > contours;

void generate() {
	// img = imread("map.png",0);
	// Mat t = imread("map.png",0);

	cout << "Generating the configuration space..." << endl;
	int angles[5] = {0,45,90,135,180};

	for(int angle = 0;angle<5;angle++) {
		stringstream ss;
		ss << angles[angle];
		string str = ss.str();

		Mat out = imread(str +  ".png",0);
		imshow("A",out);
		waitKey(10);
		grid[angles[angle]] = out;
		imshow("A",grid[angles[angle]]);
		waitKey(0);

	}

	cout << "Done" << endl;	
	// imshow("B",img);
	// imshow("A",c_spaces[45]);
	// waitKey(0);
}

#endif
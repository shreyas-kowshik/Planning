/*
Generate configuration space and apply A* search in 3D (x,y,theta)
*/

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
#include "c_test.h"
using namespace cv;
using namespace std;
# define INF 0x3f3f3f3f
#define dilation_frequency 2
#define resize_factor 0.5

//Note : NOT Implemented as a class as it was easier to order elements in the set
typedef pair<int, pair<int, int> > Pair;
typedef pair<int, int> two_Pair;
typedef pair<double, pair<int, pair<int, int> > > pPair;
Mat  grid[360];
int angles[5] = {0,45,90,135,180};

// A structure to hold the details of every cell in the grid
struct cell
{
    int parent_i, parent_j,parent_theta;
    // f = g + h
    double f, g, h;
};

class ImageProcessor {
public:
    Mat image;
    ImageProcessor(Mat image) {
        this->image = image;
    }

    //extracts a point from given image by color extraction
    two_Pair getCenter(int colorArray[6]) {
        int i_sum = 0;
        int j_sum = 0;
        int count = 0;

        for(int i = 0;i<image.rows;i++) {
            for(int j = 0;j<image.cols;j++) {
                if(image.at<Vec3b>(i,j)[0] >= colorArray[0] & image.at<Vec3b>(i,j)[0] <= colorArray[1] & image.at<Vec3b>(i,j)[1] >= colorArray[2] & image.at<Vec3b>(i,j)[1] <= colorArray[3] & image.at<Vec3b>(i,j)[2] >= colorArray[4] & image.at<Vec3b>(i,j)[2] <= colorArray[5]) {
                    i_sum+=i;
                    j_sum+=j;
                    count++;
                }
            }
        }
        int i = (i_sum/count);
        int j = (j_sum/count);
        return make_pair(i,j);
        }
    };

template <typename T>
class DynamicArray
{
public:
    DynamicArray(){};

    DynamicArray(int h, int rows,int cols): dArray(h, vector<vector<T> >(rows, vector<T>(cols))){}

    vector<vector<T> > & operator[](int i) 
    { 
      return dArray[i];
    }
    const vector<vector<T> > operator[] (int i) const 
    { 
      return dArray[i];
    }

private:
    vector<vector<vector <T> > > dArray;  
};





class PathPlanner {

private:
bool isValid(int k,int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL) && (k>=0) && (k < 5);
}
 
// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int row, int col,int k)
{   
    // Returns true if the cell is not blocked else false
    if (grid[angles[k]].at<uchar>(row,col) < 50)
        return (true);
    else
        return (false);
}
 
// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, int theta,Pair dest)
{
    if (theta == dest.first && row == dest.second.first && col == dest.second.second)
        return (true);
    else
        return (false);
}
 
// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, int theta,Pair dest)
{
    // Return using the distance formula
    return ((double)sqrt ((theta-dest.first)*(theta-dest.first)
                          + (row-dest.second.first)*(row-dest.second.first)
                          + (col-dest.second.second)*(col-dest.second.second)));
}
 
// A Utility Function to trace the path from the source
// to destination
void tracePath(DynamicArray<cell> cellDetails, Pair dest)
{
    int theta = dest.first;
    int row = dest.second.first;
    int col = dest.second.second;
 
    stack<Pair> Path;

    while (!(cellDetails[theta][row][col].parent_i == row
             && cellDetails[theta][row][col].parent_j == col && 
             cellDetails[theta][row][col].parent_theta == theta))
    {
        Path.push (make_pair(theta,make_pair (row, col)));
        int temp_row = cellDetails[theta][row][col].parent_i;
        int temp_col = cellDetails[theta][row][col].parent_j;
        int temp_theta = cellDetails[theta][row][col].parent_theta;
        row = temp_row;
        col = temp_col;
        theta = temp_theta;
    }
 
    Path.push (make_pair(theta,make_pair (row, col)));
    while (!Path.empty())
    {
    Mat t;
    display_img.copyTo(t);
    pair<int,pair<int,int> > p = Path.top();
    display_img.at<Vec3b>(p.second.first,p.second.second) = {255,255,0};

    cout << angles[p.first] << endl;
    //cout << p.second.first << " " << p.second.second << " " << p.first << endl;
    RotatedRect rRect = RotatedRect(Point2f(p.second.second,p.second.first), Size2f(10,20), angles[p.first]);
    Point2f vertices2f[4];
    rRect.points(vertices2f);
    Point vertices[4];

    for (int i = 0; i < 4; i++)
     vertices[i] = vertices2f[i];

    fillConvexPoly(t,vertices,4,Scalar(0,0,255));

    //cout << p.first << endl;
    //cout << "PATH DRAWN" << endl;
    imshow("PATH",t);
    waitKey(30);
    Path.pop();
  
    
    }
 
    return;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
void aStarSearch(Mat grid[360], Pair src, Pair dest,int ROW,int COL)
{   
    cout << "A_Star Search Started...Please be patient..." << endl;
    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    DynamicArray<bool> closedList = DynamicArray<bool>(5,ROW,COL);

    for(int i = 0;i<ROW;i++) {
        for(int j = 0;j<COL;j++) {
            for(int k = 0;k<5;k++){
            //cout << i << " " << j << " " << k << " " <<endl;
            closedList[k][i][j] = false;
            }
        }
    }
 
    // Declare a 2D array of structure to hold the details
    //of that cell
    DynamicArray<cell> cellDetails = DynamicArray<cell>(5,ROW,COL);
 
    int i, j,k;
 
    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
            for(k = 0;k<5;k++) {
            cellDetails[k][i][j].f = FLT_MAX;
            cellDetails[k][i][j].g = FLT_MAX;
            cellDetails[k][i][j].h = FLT_MAX;
            cellDetails[k][i][j].parent_i = -1;
            cellDetails[k][i][j].parent_j = -1;
            cellDetails[k][i][j].parent_theta = 0;

            //cout << k << " " << i << " " << j << endl;
        }
        
        }
    }
 
    // // Initialising the parameters of the starting node
    i = src.second.first, j = src.second.second, k = src.first;
    cellDetails[k][i][j].f = 0.0;
    cellDetails[k][i][j].g = 0.0;
    cellDetails[k][i][j].h = 0.0;
    cellDetails[k][i][j].parent_i = i;
    cellDetails[k][i][j].parent_j = j;
    cellDetails[k][i][j].parent_theta = k;

    //cout << cellDetails[1][286][12].f << endl;
    // /*
    //  Create an open list having information as-
    //  <f, <i, j>>
    //  where f = g + h,
    //  and i, j are the row and column index of that cell
    //  Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    //  This open list is implenented as a set of pair of pair.*/
    set<pPair> openList;
 
    // Put the starting cell on the open list and set its
    // 'f' as 0
    openList.insert(make_pair (0.0, make_pair(k,make_pair (i, j))));
 
    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;
 
    while (!openList.empty())
    {
    pPair p = *openList.begin();
 
    //     // Remove this vertex from the open list
    openList.erase(openList.begin());
 
        //Add this vertex to the open list
        i = p.second.second.first;
        j = p.second.second.second;
        int th = p.second.first;
        closedList[th][i][j] = true;
        //cout << th << endl;
        //cout << i << " " << j << " " << th << endl;
        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;
        //cout << i << " " << j << " " << angles[th] << endl;
        for(int k = -1;k <= 1;k++) {
        for(int l = -1;l<=1;l++) {
        for(int m = -1;m <= 1;m++) {
        // Check validity

        if(k == 0 && l == 0 && m == 0) continue;
        if (isValid(th+m,i+k, j+l) == true)
        {   
            //cout << closedList[th+m][i+k][j+l] << " " << isUnBlocked(i+k,j+l,th+m) <<endl;
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+k, j+l, th+m,dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[th+m][i+k][j+l].parent_i = i;
                cellDetails[th+m][i+k][j+l].parent_j = j;
                cellDetails[th+m][i+k][j+l].parent_theta = th;
                tracePath (cellDetails, dest);
                foundDest = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following

            else if (closedList[th+m][i+k][j+l] == false &&
                      isUnBlocked(i+k, j+l,th+m) == true)
            {   
                //cout << "Update check" << endl;
                if(k == 0 || l == 0)
                    gNew = cellDetails[th][i][j].g + 1.0;
                else {
                    if(m == 0)
                    gNew = cellDetails[th][i][j].g + 2.0;
                    else
                    gNew = cellDetails[th][i][j].g + 3.0;
                }

                if(angles[th] == 0)
                    if(l!=0)continue;
                if(angles[th] == 180)
                    if(l!=0)continue;
                if(angles[th] == 45)
                    if(abs(k) != 1 || abs(l) != 1 || (k*l) > 0) continue;
                if(angles[th] == 90)
                    if(k!=0)continue;
                if(angles[th] == 135)
                    if(abs(k) != 1 || abs(l) != 1 || (k*l) < 0) continue;

                hNew = calculateHValue (i+k, j+l,th+m, dest);
                fNew = gNew + hNew;
 
                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << th+m << " " << i+k << " " << j+l << endl;
                //cellDetails[1][286][12].f = 0.0f;
                if (cellDetails[th+m][i+k][j+l].f > fNew)
                {
                    openList.insert( make_pair(fNew,
                                               make_pair(th+m,make_pair(i+k, j+l))));
 
                    //Update the details of this cell
                    cellDetails[th+m][i+k][j+l].f = fNew;
                    cellDetails[th+m][i+k][j+l].g = gNew;
                    cellDetails[th+m][i+k][j+l].h = hNew;
                    cellDetails[th+m][i+k][j+l].parent_i = i;
                    cellDetails[th+m][i+k][j+l].parent_j = j;
                    cellDetails[th+m][i+k][j+l].parent_theta = th;
                }
            }
        }
        }
    }
    }
        
    }
 
    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");
    else
        cout << "Found Path";

    return;
}

public:
    Mat display_img;
    Pair src,dest;
    int ROW,COL;

    PathPlanner(Mat display_img,Pair src,Pair dest) {
        this->display_img = display_img;
        this->src = src;
        this->dest = dest;
        ROW = grid[0].rows;
        COL = grid[0].cols;
        aStarSearch(grid,this->src,this->dest,this->ROW,this->COL);
    } 

    void showPath() {
        //rectangle(display_img,Point(src.second-5,src.first-10),Point(src.second+5,src.first+10),{0,0,255},2,8,0);
        imshow("PATH",display_img);
        waitKey(0);
    }
};

int main()
{   
    generate();
    // imshow("A",grid[0]);
    // waitKey(0);
    Mat i = imread("map3.png",0);
    Mat img = imread("map3.png",0);
    //resize(i,img,Size(),resize_factor,resize_factor,INTER_LINEAR);

    Mat display_img = imread("New.png",1);
    //Mat display_img = Mat(i.rows/4,i.cols/4,CV_8UC1,Scalar(0));
    //resize(display_i,display_img,Size(),resize_factor,resize_factor,INTER_LINEAR);

    namedWindow("PATH",WINDOW_NORMAL);
    // int dilation_size = 3;
    // Mat element = getStructuringElement(MORPH_CROSS,
    //                                    Size( 2*dilation_size + 1, 2*dilation_size+1 ),
    //                                    Point( dilation_size, dilation_size ) );
    // /// Apply the dilation operation
    // for(int i = 0;i<dilation_frequency;i++)
    // dilate(img,img, element );

    ImageProcessor imgProcessor = ImageProcessor(display_img);
    
    int green[6] = {0,100,175,255,0,255};
    int red[6] = {0,50,0,250,200,255};
    Pair src = make_pair(0,imgProcessor.getCenter(green));

    // Destination is the left-most top-most corner
    Pair dest = make_pair(0,imgProcessor.getCenter(red));
    

    PathPlanner planner = PathPlanner(display_img,src,dest);
    planner.showPath();

    return(0);
}
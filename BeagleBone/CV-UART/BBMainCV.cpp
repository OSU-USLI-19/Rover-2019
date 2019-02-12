/* BBMainCV
 *
 * Copyright Leif Tsang, School of Computer Science, Oregon State University
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that source code redistributions retain this notice.
 *
 * This software is provided AS IS and it comes with no warranties of any type.
 *
 * Build with g++ -O2 `pkg-config --cflags --libs opencv` BBMainCV.cpp -o BBMainCV 
 */
 
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace std;
using namespace cv;



Mat testFrame;
int thresh = 275;
int max_thresh = 500;
RNG rng(12345);


Point CenterFinder(Mat* edges, Mat* frame)
{
    vector<Vec3f> circles;      //Vector of pairs of float vectors
    Point center;
    Vec3i temp;
    int MAX = 0;
    
    HoughCircles(*edges, circles, HOUGH_GRADIENT, 1, 
                5,              //Change this value to detect circles with different distances to each other   edges->rows/4
                100, 35,        //Thresholds
                1, 200);        //minimum and maximum size circles to detect min_radius and max_radius
    
    for( size_t i = 0; i < circles.size(); i++ )                //for each circle
    {
        Vec3i c = circles[i];              
        center = Point(c[0], c[1]);                             //Grab Center point
        int radius = c[2];                                      //Grab Radius
        circle( *frame, center, radius, Scalar(0, 255, 0), 3);  //Draw Circle
    }
    
    
    int centerCount[circles.size()];
    
    for(int i = 0; i < circles.size(); i++){
        centerCount[i] = 0;
    }
    
    if(circles.size() > 3){
        
        for(size_t i = 0; i < circles.size(); i++){
            temp = circles[i];
            
            for(size_t i = 0; i < circles.size(); i++){
                if((temp[0] + 50 > circles[i][0]) && (temp[0] - 50 < circles[i][0]))
                {
                    centerCount[i]++;
                }
            }
        }
        
    }
    
    for(int i = 0; i < circles.size(); i++){
        if(MAX < centerCount[i]){
            Vec3i c2 = circles[i];
            MAX = centerCount[i];
            center = Point(c2[0], c2[1]);
        }
    }
    
    /*
    for(int i = 0; i < circles.size(); i++){
        cout << centerCount[i] << " "; 
    }
    cout << endl;
    */
    
    imwrite("Captures/capture.png", *frame);                       //Write to PNG

    return center;
}

int main(int argc, char** argv)
{
    
    VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    
    if(!capture.isOpened()){
	    cout << "Failed to connect to the camera." << endl;
    }
    
    Mat frame, edges, cannyEdges;
    int count = 0;
    Point center;
    int frames = 1;
    
    
    struct timespec start, end;
    clock_gettime( CLOCK_REALTIME, &start );
    
    

    while(count != frames){
        capture >> frame;
        
        count++;
        
        if(frame.empty()){
		    cout << "Failed to capture an image" << endl;
		    return -1;
        }
    
        
        cvtColor(frame, edges, CV_BGR2GRAY);
        
        //GaussianBlur( edges, edges, Size(9, 9), 2, 2 );
        
        Canny( edges, cannyEdges, thresh, thresh*3, 3 );
        
        imwrite("Captures/edges.png", cannyEdges);
        
        center = CenterFinder(&cannyEdges, &frame);
    
        cout << "Frame Number:" << count << " " << "Center Point: "<< center << endl;
    }
  
    
    clock_gettime( CLOCK_REALTIME, &end );
    double difference = (end.tv_sec - start.tv_sec) + (double)(end.tv_nsec - start.tv_nsec)/1000000000.0d;
    cout << "It took " << difference << " seconds to process " << frames << " frames" << endl;
    cout << "Capturing and processing " << frames/difference << " frames per second " << endl;
    

    
    
    
    return 0;
}

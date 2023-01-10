#pragma once
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include"opencv2/flann.hpp"
#include"opencv2/xfeatures2d.hpp"
#include"opencv2/ml.hpp"
using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace cv::ml;
void DrawPoints(cv::Mat image, const std::vector<cv::Point>& points);
void DrawLine(cv::Mat image, const cv::Point& p1, const cv::Point& p2);
void DrawDelaunayTriangles(cv::Mat image, std::vector<cv::Point> points, int width, int height);
void KeyPointsToPoints(vector<KeyPoint> kpts, vector<Point> &pts);

#include "DrawDelaunay.h"
cv::Scalar dot_color{ 180.0, 244.0, 66.0 }; // cyan / green blue color
cv::Scalar line_color{ 15.0, 100.0, 15.0 }; // green
void DrawPoint(cv::Mat image, const cv::Point& point) {
	cv::circle(image, point, 5, dot_color, -1);
}
void DrawPoints(cv::Mat image, const std::vector<cv::Point>& points) {
	for (size_t i = 0; i < points.size(); i++)
		DrawPoint(image, points[i]);
}
void DrawLine(cv::Mat image, const cv::Point& p1, const cv::Point& p2) {
	cv::line(image, p1, p2, line_color, 2, 4);
}
void DrawDelaunayTriangles(cv::Mat image, std::vector<cv::Point> points, int width, int height) {
	cv::Rect screen_space{ 0,0, width, height };
	Subdiv2D subdiv(screen_space);
	for (size_t i = 0; i < points.size(); i++) {
		DrawPoint(image, points[i]); // optimize to avoid iterating through points twice
		subdiv.insert(points[i]);
	}
	vector<cv::Vec6f> triangles;
	subdiv.getTriangleList(triangles);
	for (size_t i = 0; i < triangles.size(); i++) {
		Vec6f triangle = triangles[i];
		Point pt1{ cvRound(triangle[0]), cvRound(triangle[1]) };
		Point pt2{ cvRound(triangle[2]), cvRound(triangle[3]) };
		Point pt3{ cvRound(triangle[4]), cvRound(triangle[5]) };
		if (screen_space.contains(pt1) && screen_space.contains(pt2) && screen_space.contains(pt3))
		{
			DrawLine(image, pt1, pt2);
			DrawLine(image, pt2, pt3);
			DrawLine(image, pt3, pt1);
		}
	}
}
void KeyPointsToPoints(vector<KeyPoint> kpts, vector<Point> &pts)
{
	for (int i = 0; i < kpts.size(); i++)
	{
		pts.push_back(kpts[i].pt);
	}
}
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <iostream>
#include <vector>

class CameraCalibrator
{
public:
	void setFilename();
	void addPoints();
	void doCalibration(cv::Mat &cameraMatrix, cv::Mat &dist);

private:
	std::vector<std::string> _filenames;
	std::vector<std::vector<cv::Point2f>> _srcPoints;
	std::vector<std::vector<cv::Point3f>> _dstPoints;
	cv::Size _imageSize;
};

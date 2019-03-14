#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <iostream>
#include <vector>

class LaneDetector
{
public:
    LaneDetector(const cv::Mat& originalImage, const cv::Mat& perspectiveMatrix);
    ~LaneDetector();

    void detectLanes();
	cv::Mat getHSL();
	cv::Mat getEdgeDetectResult();
	cv::Mat getWarpEdgeDetectResult();
	cv::Mat getWhiteYellow();
	cv::Mat getMergeImage();
	cv::Mat getHistogramImage();
	cv::Mat getMaskImage();
	cv::Mat getWarpImage();
	cv::Mat getWarpMask();
	cv::Mat getFinalResult();

    float getLaneCenterDistance();
    void setInputImage(cv::Mat& image);

private:
	cv::Mat _perspectiveMatrix;

	cv::Mat _originalImage; // входное изображение
	cv::Mat _edgeImage; // результат применени€ детектора  энни. TODO: —обель
	cv::Mat _hsl;
	cv::Mat _warpOriginalImage;
	cv::Mat _warpEdgeImage;

	cv::Mat _whiteYellow;
	cv::Mat _mergeImage;
	cv::Mat _mergeImageRGB;

	cv::Mat _histogramImage; // визуализаци€ гистограммы
	cv::Mat _maskImage; // маска, котора€ с прозрачностью будет накладыватьс€ на дорогу(син€€)
	cv::Mat _maskImageWarp;

	cv::Mat _finalResult;

	std::vector<int> _histogram;

	std::vector<cv::Point2f> _laneL;
	std::vector<cv::Point2f> _laneR;

	std::vector<cv::Point2f> _curvePointsL;
	std::vector<cv::Point2f> _curvePointsR;

	int _laneLeftCount;
	int _laneRightCount;

	int _midPoint;
	int _midHeight;

	int _leftLanePos;
	int _rightLanePos;

	short _initRecordCount;
	const int _blockNum;

	int _stepY;
	const int _windowSize;

	Eigen::Vector3d _curveCoefLeft;
	Eigen::Vector3d _curveCoefRight;
	Eigen::Vector3d _curveCoefRecordLeft[5];
	Eigen::Vector3d _curveCoefRecordRight[5];

	int _recordCounter;
	bool failDetectFlag;

	void calculateHistogram();

	void boundaryDetection();

	void laneSearch(const int &lanePos, std::vector<cv::Point2f> &_line, 
		int &lanecount, std::vector<cv::Point2f> &curvePoints, char dir);

	bool laneCoefficientsEstimate();
	void laneFitting();

	cv::Mat dir_thresh(cv::Mat &src, double mag_min, double mag_max, bool is_gray);
	auto convert_hls(cv::InputArray image);
	auto select_white_yellow(cv::InputArray image);
};

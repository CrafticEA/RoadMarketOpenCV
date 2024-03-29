#include "stdafx.h"
#include "LaneDetector.h"

LaneDetector::LaneDetector(const cv::Mat& originalImage, const cv::Mat& perspectiveMatrix)
	:_originalImage(originalImage), //�������� �������� � ������� ��������
	_perspectiveMatrix(perspectiveMatrix),//������������� �������
	_blockNum(9),//����� ����� ��������
	_windowSize(150),//������ ����
	_recordCounter(0),//������� ������ ��� ���������� �� ����
	_initRecordCount(0),//???
	failDetectFlag(true)//�������� �� ����� �������������
{
	_histogram.resize(originalImage.size().width);//������ ����������� �� ������ ��������
	_midPoint = originalImage.size().width >> 1;//������� ����� (����� �������)
	_midHeight = (int)(originalImage.size().height * 0.55);//�� ������

	_stepY = _originalImage.size().height / _blockNum;//��� �� � ��� ������ �����

	//Eigen::Vector3d initV; //��������� �������� ��� �������� ���������� �����
	//initV << 0, 0, 0;//��� ������

	//for (int i = 0; i < 5; i++)
	//	_curveCoefRecordLeft[i] = initV;//���������� �������
}

LaneDetector::~LaneDetector() {}

// �������������� � ������ �������� ������
auto LaneDetector::convert_hls(cv::InputArray image)
{
	cv::Mat res = cv::Mat(image.rows(), image.cols(), 8, 3);// ������� ������ �������
	cv::cvtColor(image, res, cv::COLOR_BGR2HLS);//������������ ����� � BRG � HLS
	_hsl = res;
	return res;
}

// �������� ����� � ������ ���� �� �����
auto LaneDetector::select_white_yellow(cv::InputArray image)
{
	auto converted = convert_hls(image);

	// ��� ������ �����
	cv::Vec3b l1(0, 200, 0);//������ ������ ����� (������)
	cv::Vec3b u1(255, 255, 255);//�������
	cv::Mat white_mask; //����� �� ������ �����
	cv::inRange(converted, l1, u1, white_mask);// �������� ������ � ���. �����

	// ��� ������� �����
	cv::Vec3b l2(10, 0, 100); //�� �� ����� ��� �������
	cv::Vec3b u2(40, 255, 255);
	cv::Mat yellow_mask;
	cv::inRange(converted, l2, u2, yellow_mask);

	// ����������� ���������� �����
	cv::Mat mask, masked;
	cv::bitwise_or(white_mask, yellow_mask, mask);//���������� �����
	cv::bitwise_and(image, image, masked, mask);

	return masked;
}

//cv::Mat LaneDetector::dir_thresh(cv::Mat& src, double mag_min, double mag_max, bool is_gray)
//{
//	cv::Mat gray;
//
//	if (!is_gray)
//		cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);
//	else
//		gray = src;
//
//	cv::Mat grad_x, grad_y;
//	cv::Sobel(gray, grad_x, CV_64F, 1, 0, 3);
//	cv::Sobel(gray, grad_y, CV_64F, 0, 1, 3);
//	cv::Mat abs_grad_dir;
//	cv::phase(grad_x, grad_y, abs_grad_dir);
//
//	cv::Mat bin;
//	cv::threshold(src, bin, mag_min, mag_max, cv::THRESH_BINARY);
//	return bin;
//}

// ����������, ������ ��� ������, ������� ������� � ������ �������
void LaneDetector::detectLanes()
{
	_whiteYellow = select_white_yellow(_originalImage);//����-������ ������
	cv::Mat grayscale;//������� ������ �������

	cvtColor(_whiteYellow, grayscale, cv::COLOR_RGB2GRAY);//��������� � ������� ������
	Canny(grayscale, _edgeImage, 50, 150, 3);//�������� �����

	warpPerspective(_edgeImage, _warpEdgeImage, _perspectiveMatrix, _edgeImage.size());//������������ � �����������
	inRange(_warpEdgeImage, cv::Scalar(1), cv::Scalar(255), _warpEdgeImage);//�����-����� �����
	warpPerspective(_originalImage, _warpOriginalImage, _perspectiveMatrix, _originalImage.size());//�������� � �����������

	_mergeImage = _warpEdgeImage;
	cvtColor(_mergeImage, _mergeImageRGB, CV_GRAY2RGB);//���� � ���

	calculateHistogram();//������������ �����������

	boundaryDetection();//����������� ������

	laneSearch(_leftLanePos, _laneL, _laneLeftCount, _curvePointsL, 'L');//����� �����
	laneSearch(_rightLanePos, _laneR, _laneRightCount, _curvePointsR, 'R');//

	laneCoefficientsEstimate();//������� ����������� ������ �����
	laneFitting();//������� �����

	warpPerspective(_maskImage, _maskImageWarp, _perspectiveMatrix, _maskImage.size(), cv::WARP_INVERSE_MAP);//����������� ����� � �������� � �����
}

// ������� �����������
void LaneDetector::calculateHistogram()
{
	_histogram.clear();//�������

	for (int i = 0; i < _mergeImage.size().width; i++)
	{
		cv::Mat ROI = _mergeImage(cv::Rect(i, _originalImage.size().height - _midHeight - 1, 1, _midHeight));//����� ��������� ������� ���� �����������
		cv::Mat dst;//���������� ��������� ����
		cv::divide(255, ROI, dst);//???
		_histogram.push_back((int)(sum(dst)[0]));//������������ ���� ��������
	}

	int maxValue = 0;//������������ ���� � �����������
	maxValue = (*max_element(_histogram.begin(), _histogram.end()));//�������

	_histogramImage.create(maxValue, _histogram.size(), CV_8UC3);//���	���� ��������
	_histogramImage = cv::Scalar(255, 255, 255);//������� �����

	for (size_t i = 0; i < _histogram.size(); i++)
		cv::line(_histogramImage, cv::Point2f(i, (maxValue - _histogram.at(i))),//������ ���� �� �����������
			cv::Point2f(i, maxValue), cv::Scalar(0, 0, 0), 1);
}

// ����������� ������
void LaneDetector::boundaryDetection()
{
	std::vector<int>::iterator maxLeftPtr;//
	maxLeftPtr = max_element(_histogram.begin(), _histogram.begin() + _midPoint - 1);//������� ������������ �������� ��� �����
	int maxL = *maxLeftPtr;//���������� ���������
	_leftLanePos = distance(_histogram.begin(), maxLeftPtr);//���������� �� ����(������� �������)

	std::vector<int>::iterator maxRightPtr;//��� ������
	maxRightPtr = max_element(_histogram.begin() + _midPoint, _histogram.end());
	int maxR = *maxRightPtr;
	_rightLanePos = distance(_histogram.begin(), maxRightPtr);

	if (_initRecordCount < 5 || failDetectFlag == true)//������ ������� ����� �� ������������� �������� �� �����������
	{
		cv::line(_mergeImageRGB, cv::Point2f(_leftLanePos, 0),
			cv::Point2f(_leftLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);

		cv::line(_mergeImageRGB, cv::Point2f(_rightLanePos, 0),
			cv::Point2f(_rightLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);
	}
}

// ��� �� ������ ������(��. ����������� � ������� ������ �� wiki)
void LaneDetector::laneSearch(const int& lanePos, std::vector<cv::Point2f>& _line,
	int& laneCount, std::vector<cv::Point2f>& curvePoints, char dir)
{
	_line.clear();

	const int skipStep = 4;

	int nextPosX = lanePos;

	int upperLeftX = 0, upperLeftY = 0;
	int bottomRightX = 0, bottomRightY = 0;

	int windowSize = _windowSize;

	int stepY = _stepY;

	int sumX = 0;
	int xcounter = 0;

	laneCount = 0;

	if (_initRecordCount < 5 || failDetectFlag == true)
		for (int i = 0; i < _blockNum; i++)
		{
			windowSize = _windowSize;

			upperLeftX = nextPosX - (_windowSize >> 1); // The x coordinate of the upper left point.
			upperLeftY = _stepY * (_blockNum - i - 1); // The y coordinate of the upper left point.
			bottomRightX = upperLeftX + _windowSize; // The x coordinate of the bottom right point.
			bottomRightY = upperLeftY + _stepY - 1; // The y coordinate of the bottom right point.

			if (upperLeftX < 0)
			{
				upperLeftX = 0;
				bottomRightX = upperLeftX + _windowSize;
			}

			if (bottomRightX > _mergeImage.size().width - 1)
			{
				windowSize = _windowSize + (_mergeImage.size().width - 1 - bottomRightX);
				bottomRightX = (_mergeImage.size().width - 1);
				upperLeftX += (_mergeImage.size().width - 1 - bottomRightX);
			}

			if (bottomRightX - upperLeftX > 0 && bottomRightX >= 0 && upperLeftX >= 0)
			{
				sumX = 0;
				xcounter = 0;
				uchar* matPtr;

				for (int j = upperLeftY; j <= bottomRightY; j += skipStep)
				{
					matPtr = _mergeImage.data + (j * _mergeImage.size().width);

					for (int k = upperLeftX; k <= bottomRightX; k += skipStep)
						if (*(matPtr + k) == 255)
						{
							sumX += k;
							++xcounter;
						}
				}

				if (xcounter != 0)
					sumX /= xcounter;
				else
					sumX = nextPosX;

				nextPosX = sumX;

				upperLeftX = ((nextPosX - (_windowSize >> 1)) > 0) ? (nextPosX - (_windowSize >> 1)) : 0;

				bottomRightX = ((upperLeftX + _windowSize) < (_mergeImage.size().width)) ?
					(upperLeftX + _windowSize) : (_mergeImage.size().width - 1);

				if (bottomRightX - upperLeftX > 0 && bottomRightX >= 0 && upperLeftX >= 0)
					for (int j = upperLeftY; j <= bottomRightY; j += skipStep)
					{
						matPtr = _mergeImage.data + (j * _mergeImage.size().width);
						for (int k = upperLeftX; k <= bottomRightX; k += skipStep)
							if (*(matPtr + k) == 255)
							{
								laneCount++;
								_line.push_back(cv::Point2f(k, j));
							}
					}

				rectangle(_mergeImageRGB, cv::Point2f(upperLeftX, upperLeftY),
					cv::Point2f(bottomRightX, bottomRightY), cv::Scalar(255, 0, 0), 5);
			}
		}

	else
	{
		uchar* matPtr;
		int xtemp;
		for (int i = 0; i < _mergeImage.size().height; i++)
		{
			matPtr = _mergeImage.data + (i * _mergeImage.size().width);
			for (int j = -50; j <= 50; j += 3)
			{
				xtemp = (curvePoints[i].x + j);
				if (xtemp >= 0 && xtemp < _mergeImage.size().width)
				{
					if (*(matPtr + xtemp) == 255)
					{
						laneCount++;
						_line.push_back(cv::Point2f(xtemp, i));
						if (i >= (_mergeImage.size().height / 2))
						{
							sumX += xtemp;
							xcounter++;
						}

					}
					_mergeImageRGB.at<cv::Vec3b>(i, xtemp)[0] = 0;
					_mergeImageRGB.at<cv::Vec3b>(i, xtemp)[1] = 255;
					_mergeImageRGB.at<cv::Vec3b>(i, xtemp)[2] = 255;
				}
			}
		}
		sumX /= xcounter;
		if (sumX > 0 && sumX < _mergeImageRGB.size().width)
		{
			if (dir == 'L')
			{
				_leftLanePos = sumX;
				line(_mergeImageRGB, cv::Point2f(_leftLanePos, 0),
					cv::Point2f(_leftLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);
			}
			else
			{
				_rightLanePos = sumX;
				line(_mergeImageRGB, cv::Point2f(_rightLanePos, 0),
					cv::Point2f(_rightLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);
			}
		}
		else
		{
			if (dir == 'L')
				line(_mergeImageRGB, cv::Point2f(_leftLanePos, 0),
					cv::Point2f(_leftLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);

			else
				line(_mergeImageRGB, cv::Point2f(_rightLanePos, 0),
					cv::Point2f(_rightLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);
		}
	}
}

// ��������� ������������ ������ � ������� ������������ ����������
bool LaneDetector::laneCoefficientsEstimate()
{
	int countThreshold = 300;//�������� �� ������ �� ��������� ������

	if (_laneLeftCount > countThreshold && _laneRightCount > countThreshold)//�������� ����� ����� �� �����
	{
		Eigen::VectorXd xValueL(_laneLeftCount);//
		Eigen::VectorXd xValueR(_laneRightCount);//
		Eigen::MatrixXd leftMatrix(_laneLeftCount, 3);//
		Eigen::MatrixXd rightMatrix(_laneRightCount, 3);//

		for (int i = 0; i < _laneLeftCount; i++)
		{
			xValueL(i) = _laneL[i].x;//���������������� ������� ��� ����� �����
			leftMatrix(i, 0) = pow(_laneL[i].y, 2);
			leftMatrix(i, 1) = _laneL[i].y;
			leftMatrix(i, 2) = 1;
		}

		for (int i = 0; i < _laneRightCount; i++)//���������������� ������� ��� ������ �����
		{
			xValueR(i) = _laneR[i].x;
			rightMatrix(i, 0) = pow(_laneR[i].y, 2);
			rightMatrix(i, 1) = _laneR[i].y;
			rightMatrix(i, 2) = 1;
		}

		_curveCoefLeft = (leftMatrix.transpose() * leftMatrix).ldlt().solve(leftMatrix.transpose() * xValueL);//��������� ������������ ������ ��� �����
		_curveCoefRight = (rightMatrix.transpose() * rightMatrix).ldlt().solve(rightMatrix.transpose() * xValueR);//��� ������

		_curveCoefRecordLeft[_recordCounter] = _curveCoefLeft;//���������� ��� ������������������ ��� ������������� ����������� �� ���� � ����������
		_curveCoefRecordRight[_recordCounter] = _curveCoefRight;//��� ������
		_recordCounter = (_recordCounter + 1) % 5;//������� ������� ��� �������� ������������ �� ����

		if (_initRecordCount < 5)
			_initRecordCount++;//��������� ������� ������

		failDetectFlag = false;//� ������ ������ ������ ����

		return true;//�����. ������
	}
	else
	{
		std::cerr << "[Lane Detection] There is no enough detected road marks.";//������ �� ���������� �����
		failDetectFlag = true;//������ ����
		return false;
	}
}

// �������� �����(��������������, ������� ����������� �� ������������ �����������)
void LaneDetector::laneFitting()
{
	_maskImage.create(_mergeImage.size().height, _mergeImage.size().width, CV_8UC3);//������� ������� � ������� ����� ������� �����
	_maskImage = cv::Scalar(0, 0, 0);// ��������� ������
	_curvePointsL.clear();//������� ���� ��� �����
	_curvePointsR.clear();//� ��� ������

	if (_initRecordCount == 5) //����� ����������� ���� ������
	{
		_curveCoefLeft = (_curveCoefRecordLeft[0]//���������� ����������� ����� � ����� �� ���� ��� �����
			+ _curveCoefRecordLeft[1]
			+ _curveCoefRecordLeft[2]
			+ _curveCoefRecordLeft[3]
			+ _curveCoefRecordLeft[4]) / 5;
		_curveCoefRight = (_curveCoefRecordRight[0]//��� ������
			+ _curveCoefRecordRight[1]
			+ _curveCoefRecordRight[2]
			+ _curveCoefRecordRight[3]
			+ _curveCoefRecordRight[4]) / 5;
	}

	int xLeft, xRight;//
	for (int i = 0; i < _mergeImage.size().height; i++)
	{
		xLeft = pow(i, 2) * _curveCoefLeft(0) + i * _curveCoefLeft(1) + _curveCoefLeft(2);//������ �������. ��-��
		xRight = pow(i, 2) * _curveCoefRight(0) + i * _curveCoefRight(1) + _curveCoefRight(2);//� ��� ������

		if (xLeft < 0)//��������� �� �������
			xLeft = 0;//�� ������ �����

		if (xLeft >= _mergeImage.size().width)//�� ������ ������
			xLeft = _mergeImage.size().width - 1;

		if (xRight < 0)//��� ������ ����������
			xRight = 0;

		if (xRight >= _mergeImage.size().width)
			xRight = _mergeImage.size().width - 1;

		_curvePointsL.push_back(cv::Point2f(xLeft, i));//��������������� ���������� � ������ ��������� �����������. �-��
		_curvePointsR.push_back(cv::Point2f(xRight, i));
	}

	cv::Mat curveL(_curvePointsL, true);//������� ������, ���������� ����������
	curveL.convertTo(curveL, CV_32S);//������������ � 32 ������ �����
	polylines(_maskImage, curveL, false, cv::Scalar(0, 255, 0), 20, CV_AA);//������ ����� �������

	cv::Mat curveR(_curvePointsR, true);//�� �� ����� ��� ������
	curveR.convertTo(curveR, CV_32S);
	polylines(_maskImage, curveR, false, cv::Scalar(0, 255, 0), 20, CV_AA);

	uchar* matPtr;// ��������� �� �������
	for (int i = 0; i < _maskImage.size().height; i++)
	{
		matPtr = _maskImage.data + i * _maskImage.size().width * 3;//��������� ��������� ��������� ������
		for (int j = _curvePointsL[i].x; j <= _curvePointsR[i].x; j++)
		{
			*(matPtr + j * 3) = 255;//����� 
			*(matPtr + j * 3 + 1) = 150;//�������
			*(matPtr + j * 3 + 2) = 0;//�������
		}
	}
}
//HSV HSL
cv::Mat LaneDetector::getHSL()
{
	return _hsl;
}

// ��������� ������������� ������ �����
cv::Mat LaneDetector::getEdgeDetectResult()
{
	return _edgeImage;
}

// ��������� ������������� ������ ����� � ������������� ��������
cv::Mat LaneDetector::getWarpEdgeDetectResult()
{
	return _warpEdgeImage;
}

// ��������� ����� ����-������� �������
cv::Mat LaneDetector::getWhiteYellow()
{
	return _whiteYellow;
}

cv::Mat LaneDetector::getMergeImage()
{
	return _mergeImageRGB;
}

// �������� ����������� �����������
cv::Mat LaneDetector::getHistogramImage()
{
	return _histogramImage;
}

// �������� ����������� �����
cv::Mat LaneDetector::getMaskImage()
{
	return _maskImage;
}

// ������ ��� ��������� � ������������� ��������
cv::Mat LaneDetector::getWarpImage()
{
	return _warpOriginalImage;
}

// �������� ������� ����� � ������������� ��������
cv::Mat LaneDetector::getWarpMask()
{
	return _maskImageWarp;
}

// ��������� ���������� �����(� ���������� ������)
cv::Mat LaneDetector::getFinalResult()
{
	addWeighted(_maskImageWarp, 0.5, _originalImage, 1, 0, _finalResult); // ����������� ����� � ������������� (0,5)
	return _finalResult;
}

// ������ ������� ����
void LaneDetector::setInputImage(cv::Mat& image)
{
	_originalImage = image.clone(); // ������ ����� ���������
}

// �������� ������������ ������
float LaneDetector::getLaneCenterDistance()
{
	float laneCenter = (_rightLanePos - _leftLanePos) / 2.0 + _leftLanePos; // �������� ����� ��� �������
	float imageCenter = _mergeImageRGB.size().width / 2.0; // ����� ����������� �� �

	float result;

	result = (laneCenter - imageCenter) * 3.5 / 600; // 3.5 ����� � 600 ��������
	return result;
}
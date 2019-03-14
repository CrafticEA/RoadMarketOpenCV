#include "stdafx.h"
#include "LaneDetector.h"

LaneDetector::LaneDetector(const cv::Mat& originalImage, const cv::Mat& perspectiveMatrix)
	:_originalImage(originalImage), //копируем картинку с которой работаем
	_perspectiveMatrix(perspectiveMatrix),//перспективная матрица
	_blockNum(9),//блоки линии дорожной
	_windowSize(150),//размер окна
	_recordCounter(0),//счетчик кадров для усреднения по пяти
	_initRecordCount(0),//???
	failDetectFlag(true)//проверка на этапе аппроксимации
{
	_histogram.resize(originalImage.size().width);//размер гистограммы по ширине картинки
	_midPoint = originalImage.size().width >> 1;//среднюю точку (делим пополам)
	_midHeight = (int)(originalImage.size().height * 0.55);//по высоте

	_stepY = _originalImage.size().height / _blockNum;//шаг по у для поиска линий

	//Eigen::Vector3d initV; //начальное значение для векторов усреднения линий
	//initV << 0, 0, 0;//сам вектор

	//for (int i = 0; i < 5; i++)
	//	_curveCoefRecordLeft[i] = initV;//закидываем вектора
}

LaneDetector::~LaneDetector() {}

// преобразование в другой цветовой формат
auto LaneDetector::convert_hls(cv::InputArray image)
{
	cv::Mat res = cv::Mat(image.rows(), image.cols(), 8, 3);// создаем пустая матрица
	cv::cvtColor(image, res, cv::COLOR_BGR2HLS);//конвертируем цвета с BRG в HLS
	_hsl = res;
	return res;
}

// выбирает белый и желтый цвет на кадре
auto LaneDetector::select_white_yellow(cv::InputArray image)
{
	auto converted = convert_hls(image);

	// для белого цвета
	cv::Vec3b l1(0, 200, 0);//вектор белого цвета (нижняя)
	cv::Vec3b u1(255, 255, 255);//верхняя
	cv::Mat white_mask; //маска по белому цвету
	cv::inRange(converted, l1, u1, white_mask);// собирает вектор с исп. масок

	// для желтого цвета
	cv::Vec3b l2(10, 0, 100); //то же самое для желтого
	cv::Vec3b u2(40, 255, 255);
	cv::Mat yellow_mask;
	cv::inRange(converted, l2, u2, yellow_mask);

	// накладываем полученную маску
	cv::Mat mask, masked;
	cv::bitwise_or(white_mask, yellow_mask, mask);//объединяем маски
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

// собственно, делает всю работу, вызывая функции в нужном порядке
void LaneDetector::detectLanes()
{
	_whiteYellow = select_white_yellow(_originalImage);//бело-желтый фильтр
	cv::Mat grayscale;//оттенок серого матрица

	cvtColor(_whiteYellow, grayscale, cv::COLOR_RGB2GRAY);//переводим в оттенок серого
	Canny(grayscale, _edgeImage, 50, 150, 3);//оператор канни

	warpPerspective(_edgeImage, _warpEdgeImage, _perspectiveMatrix, _edgeImage.size());//обработанный в перспективе
	inRange(_warpEdgeImage, cv::Scalar(1), cv::Scalar(255), _warpEdgeImage);//Черно-белая маска
	warpPerspective(_originalImage, _warpOriginalImage, _perspectiveMatrix, _originalImage.size());//оригинал в перспективе

	_mergeImage = _warpEdgeImage;
	cvtColor(_mergeImage, _mergeImageRGB, CV_GRAY2RGB);//грей в ргб

	calculateHistogram();//подсчитываем гистограмму

	boundaryDetection();//обнаружение границ

	laneSearch(_leftLanePos, _laneL, _laneLeftCount, _curvePointsL, 'L');//поиск линий
	laneSearch(_rightLanePos, _laneR, _laneRightCount, _curvePointsR, 'R');//

	laneCoefficientsEstimate();//находим коэфициенты кривой линии
	laneFitting();//создаем маску

	warpPerspective(_maskImage, _maskImageWarp, _perspectiveMatrix, _maskImage.size(), cv::WARP_INVERSE_MAP);//преобразуем маску в оригинал к видео
}

// подсчет гистограммы
void LaneDetector::calculateHistogram()
{
	_histogram.clear();//очищаем

	for (int i = 0; i < _mergeImage.size().width; i++)
	{
		cv::Mat ROI = _mergeImage(cv::Rect(i, _originalImage.size().height - _midHeight - 1, 1, _midHeight));//Берем отдельный столбец ориг изображения
		cv::Mat dst;//записываем результат сюда
		cv::divide(255, ROI, dst);//???
		_histogram.push_back((int)(sum(dst)[0]));//суммирование всех пикселей
	}

	int maxValue = 0;//максимальное знач в гистограмме
	maxValue = (*max_element(_histogram.begin(), _histogram.end()));//находим

	_histogramImage.create(maxValue, _histogram.size(), CV_8UC3);//соз	даем картинку
	_histogramImage = cv::Scalar(255, 255, 255);//заливка белым

	for (size_t i = 0; i < _histogram.size(); i++)
		cv::line(_histogramImage, cv::Point2f(i, (maxValue - _histogram.at(i))),//рисуем знач на гистограмме
			cv::Point2f(i, maxValue), cv::Scalar(0, 0, 0), 1);
}

// обнаружение границ
void LaneDetector::boundaryDetection()
{
	std::vector<int>::iterator maxLeftPtr;//
	maxLeftPtr = max_element(_histogram.begin(), _histogram.begin() + _midPoint - 1);//находим максимальное значение для левой
	int maxL = *maxLeftPtr;//записываем результат
	_leftLanePos = distance(_histogram.begin(), maxLeftPtr);//расстояние от края(находим позицию)

	std::vector<int>::iterator maxRightPtr;//для правой
	maxRightPtr = max_element(_histogram.begin() + _midPoint, _histogram.end());
	int maxR = *maxRightPtr;
	_rightLanePos = distance(_histogram.begin(), maxRightPtr);

	if (_initRecordCount < 5 || failDetectFlag == true)//рисуем зеленые линии по максимальному значению на гистограмме
	{
		cv::line(_mergeImageRGB, cv::Point2f(_leftLanePos, 0),
			cv::Point2f(_leftLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);

		cv::line(_mergeImageRGB, cv::Point2f(_rightLanePos, 0),
			cv::Point2f(_rightLanePos, _mergeImageRGB.size().height), cv::Scalar(0, 255, 0), 10);
	}
}

// тут мы строим кривые(см. приближение с помощью кривых на wiki)
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

// вычисляем коэффициенты кривой с помощью сингулярного разложения
bool LaneDetector::laneCoefficientsEstimate()
{
	int countThreshold = 300;//является ли линией по заданному порогу

	if (_laneLeftCount > countThreshold && _laneRightCount > countThreshold)//проверка обеих линий на порог
	{
		Eigen::VectorXd xValueL(_laneLeftCount);//
		Eigen::VectorXd xValueR(_laneRightCount);//
		Eigen::MatrixXd leftMatrix(_laneLeftCount, 3);//
		Eigen::MatrixXd rightMatrix(_laneRightCount, 3);//

		for (int i = 0; i < _laneLeftCount; i++)
		{
			xValueL(i) = _laneL[i].x;//аппроксимирующая функция для левой линии
			leftMatrix(i, 0) = pow(_laneL[i].y, 2);
			leftMatrix(i, 1) = _laneL[i].y;
			leftMatrix(i, 2) = 1;
		}

		for (int i = 0; i < _laneRightCount; i++)//аппроксимирующая функция для правой линии
		{
			xValueR(i) = _laneR[i].x;
			rightMatrix(i, 0) = pow(_laneR[i].y, 2);
			rightMatrix(i, 1) = _laneR[i].y;
			rightMatrix(i, 2) = 1;
		}

		_curveCoefLeft = (leftMatrix.transpose() * leftMatrix).ldlt().solve(leftMatrix.transpose() * xValueL);//вычисляем коэффициенты кривых для левой
		_curveCoefRight = (rightMatrix.transpose() * rightMatrix).ldlt().solve(rightMatrix.transpose() * xValueR);//для правой

		_curveCoefRecordLeft[_recordCounter] = _curveCoefLeft;//записываем для последовательности для использования усредненной по пяти в дальнейшем
		_curveCoefRecordRight[_recordCounter] = _curveCoefRight;//для правой
		_recordCounter = (_recordCounter + 1) % 5;//остаток деления для подсчета усредненного по пяти

		if (_initRecordCount < 5)
			_initRecordCount++;//обновляем счетчик кадров

		failDetectFlag = false;//в случае успеха фолсим флаг

		return true;//возвр. трушку
	}
	else
	{
		std::cerr << "[Lane Detection] There is no enough detected road marks.";//ошибка об отсутствии линии
		failDetectFlag = true;//трушим флаг
		return false;
	}
}

// создание маски(полупрозрачной, которую накладываем на оригинальное изображение)
void LaneDetector::laneFitting()
{
	_maskImage.create(_mergeImage.size().height, _mergeImage.size().width, CV_8UC3);//создаем матрицу в которой будем хранить маску
	_maskImage = cv::Scalar(0, 0, 0);// заполняем нулями
	_curvePointsL.clear();//очищаем коэф для левой
	_curvePointsR.clear();//и для правой

	if (_initRecordCount == 5) //после прохождения пяти кадров
	{
		_curveCoefLeft = (_curveCoefRecordLeft[0]//суммиируем коэфициенты кривы и делим на пять для левой
			+ _curveCoefRecordLeft[1]
			+ _curveCoefRecordLeft[2]
			+ _curveCoefRecordLeft[3]
			+ _curveCoefRecordLeft[4]) / 5;
		_curveCoefRight = (_curveCoefRecordRight[0]//для правой
			+ _curveCoefRecordRight[1]
			+ _curveCoefRecordRight[2]
			+ _curveCoefRecordRight[3]
			+ _curveCoefRecordRight[4]) / 5;
	}

	int xLeft, xRight;//
	for (int i = 0; i < _mergeImage.size().height; i++)
	{
		xLeft = pow(i, 2) * _curveCoefLeft(0) + i * _curveCoefLeft(1) + _curveCoefLeft(2);//решаем аппрокс. ур-ие
		xRight = pow(i, 2) * _curveCoefRight(0) + i * _curveCoefRight(1) + _curveCoefRight(2);//и для правой

		if (xLeft < 0)//проверяем на границы
			xLeft = 0;//по ширине влево

		if (xLeft >= _mergeImage.size().width)//по ширине вправо
			xLeft = _mergeImage.size().width - 1;

		if (xRight < 0)//для правой аналогично
			xRight = 0;

		if (xRight >= _mergeImage.size().width)
			xRight = _mergeImage.size().width - 1;

		_curvePointsL.push_back(cv::Point2f(xLeft, i));//последовательно записываем в вектор результат аппроксимир. ф-ии
		_curvePointsR.push_back(cv::Point2f(xRight, i));
	}

	cv::Mat curveL(_curvePointsL, true);//матрица кривой, записываем результаты
	curveL.convertTo(curveL, CV_32S);//конвертируем в 32 битный канал
	polylines(_maskImage, curveL, false, cv::Scalar(0, 255, 0), 20, CV_AA);//рисуем линии зеленым

	cv::Mat curveR(_curvePointsR, true);//то же самое для правой
	curveR.convertTo(curveR, CV_32S);
	polylines(_maskImage, curveR, false, cv::Scalar(0, 255, 0), 20, CV_AA);

	uchar* matPtr;// указатель на позицию
	for (int i = 0; i < _maskImage.size().height; i++)
	{
		matPtr = _maskImage.data + i * _maskImage.size().width * 3;//заполняем построчно выбранным цветом
		for (int j = _curvePointsL[i].x; j <= _curvePointsR[i].x; j++)
		{
			*(matPtr + j * 3) = 255;//синий 
			*(matPtr + j * 3 + 1) = 150;//зеленый
			*(matPtr + j * 3 + 2) = 0;//красный
		}
	}
}
//HSV HSL
cv::Mat LaneDetector::getHSL()
{
	return _hsl;
}

// результат распознавания границ линий
cv::Mat LaneDetector::getEdgeDetectResult()
{
	return _edgeImage;
}

// результат распознавания границ линий в перспективной проекции
cv::Mat LaneDetector::getWarpEdgeDetectResult()
{
	return _warpEdgeImage;
}

// результат после бело-желтого фильтра
cv::Mat LaneDetector::getWhiteYellow()
{
	return _whiteYellow;
}

cv::Mat LaneDetector::getMergeImage()
{
	return _mergeImageRGB;
}

// получить изображение гистограммы
cv::Mat LaneDetector::getHistogramImage()
{
	return _histogramImage;
}

// получить изображение маски
cv::Mat LaneDetector::getMaskImage()
{
	return _maskImage;
}

// дорога без обработки в перспективной проекции
cv::Mat LaneDetector::getWarpImage()
{
	return _warpOriginalImage;
}

// получить рисунок маски в перспективной проекции
cv::Mat LaneDetector::getWarpMask()
{
	return _maskImageWarp;
}

// получение финального кадра(с наложенной маской)
cv::Mat LaneDetector::getFinalResult()
{
	addWeighted(_maskImageWarp, 0.5, _originalImage, 1, 0, _finalResult); // накладываем маску с прозрачностью (0,5)
	return _finalResult;
}

// задать входной кадр
void LaneDetector::setInputImage(cv::Mat& image)
{
	_originalImage = image.clone(); // делаем копию оригинала
}

// смещение относительно центра
float LaneDetector::getLaneCenterDistance()
{
	float laneCenter = (_rightLanePos - _leftLanePos) / 2.0 + _leftLanePos; // получаем центр меж линиями
	float imageCenter = _mergeImageRGB.size().width / 2.0; // центр изображения по Х

	float result;

	result = (laneCenter - imageCenter) * 3.5 / 600; // 3.5 метра и 600 пикселей
	return result;
}
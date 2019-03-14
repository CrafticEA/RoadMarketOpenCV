#include "stdafx.h"
#include "calibration.h"

void CameraCalibrator::setFilename() //выборка файлов
{
	_filenames.clear();
	_filenames.push_back("./camera_cal/calibration1.jpg");
	_filenames.push_back("./camera_cal/calibration2.jpg");
	_filenames.push_back("./camera_cal/calibration3.jpg");
	_filenames.push_back("./camera_cal/calibration4.jpg");
	_filenames.push_back("./camera_cal/calibration5.jpg");
	_filenames.push_back("./camera_cal/calibration6.jpg");
	_filenames.push_back("./camera_cal/calibration7.jpg");
	_filenames.push_back("./camera_cal/calibration8.jpg");
	_filenames.push_back("./camera_cal/calibration9.jpg");
	_filenames.push_back("./camera_cal/calibration10.jpg");
	_filenames.push_back("./camera_cal/calibration11.jpg");
	_filenames.push_back("./camera_cal/calibration12.jpg");
	_filenames.push_back("./camera_cal/calibration13.jpg");
	_filenames.push_back("./camera_cal/calibration14.jpg");
	_filenames.push_back("./camera_cal/calibration15.jpg");
	_filenames.push_back("./camera_cal/calibration16.jpg");
	//_filenames.push_back("./camera_cal/calibration17.jpg");
	//_filenames.push_back("./camera_cal/calibration18.jpg");
	//_filenames.push_back("./camera_cal/calibration19.jpg");
	//_filenames.push_back("./camera_cal/calibration20.jpg");
}

void CameraCalibrator::addPoints() //метод калибровки
{
	std::vector<cv::Point2f> chessboardCorner; // координаты в двумерном
	std::vector<cv::Point3f> realWorldCoord; //коорд в трехмерном
	cv::Mat image; //буфериз изображ

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 9; j++)
			realWorldCoord.push_back(cv::Point3f(i, j, 0.0f)); //заполняем матрицу в трехмерном нулями

	for (int i = 0; i < _filenames.size(); i++) 
	{
		image = cv::imread(_filenames[i], CV_LOAD_IMAGE_GRAYSCALE); //считываем файл выборки
		_imageSize = image.size(); //считываем разрешение файла

		cv::findChessboardCorners(image, cv::Size(9, 6), chessboardCorner); //сиви функция для калибровки

		if (chessboardCorner.size() == 54) //если совпало кол-во клеток
		{
			_dstPoints.push_back(realWorldCoord); //добавляем координаты для реального изображения
			_srcPoints.push_back(chessboardCorner); //добавляем коорд для шахматной
		}
	}
}

void CameraCalibrator::doCalibration(cv::Mat &cameraMatrix, cv::Mat &dist) //метод калибровки
{
	setFilename();
	addPoints();

	std::vector<cv::Mat> rvecs, tvecs;
	calibrateCamera(_dstPoints, _srcPoints, _imageSize, cameraMatrix, dist, rvecs, tvecs);//метод калибровки
}
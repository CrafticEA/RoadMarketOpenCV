#include "stdafx.h"
#include "calibration.h"

void CameraCalibrator::setFilename() //������� ������
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

void CameraCalibrator::addPoints() //����� ����������
{
	std::vector<cv::Point2f> chessboardCorner; // ���������� � ���������
	std::vector<cv::Point3f> realWorldCoord; //����� � ����������
	cv::Mat image; //������� �������

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 9; j++)
			realWorldCoord.push_back(cv::Point3f(i, j, 0.0f)); //��������� ������� � ���������� ������

	for (int i = 0; i < _filenames.size(); i++) 
	{
		image = cv::imread(_filenames[i], CV_LOAD_IMAGE_GRAYSCALE); //��������� ���� �������
		_imageSize = image.size(); //��������� ���������� �����

		cv::findChessboardCorners(image, cv::Size(9, 6), chessboardCorner); //���� ������� ��� ����������

		if (chessboardCorner.size() == 54) //���� ������� ���-�� ������
		{
			_dstPoints.push_back(realWorldCoord); //��������� ���������� ��� ��������� �����������
			_srcPoints.push_back(chessboardCorner); //��������� ����� ��� ���������
		}
	}
}

void CameraCalibrator::doCalibration(cv::Mat &cameraMatrix, cv::Mat &dist) //����� ����������
{
	setFilename();
	addPoints();

	std::vector<cv::Mat> rvecs, tvecs;
	calibrateCamera(_dstPoints, _srcPoints, _imageSize, cameraMatrix, dist, rvecs, tvecs);//����� ����������
}
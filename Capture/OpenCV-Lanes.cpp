#include "stdafx.h"

#include "LaneDetector.h"
#include "calibration.h"

cv::VideoCapture videoCapture;

cv::Mat videoFrame;
cv::Mat videoFrameUndistorted;
cv::Mat videoFramePerspective;

cv::Mat _videoFrameUndistorted;

cv::Size videoSize;
cv::Mat cameraMatrix, dist;
cv::Mat perspectiveMatrix;
cv::String coordinatetext = "";

cv::Point2f perspectiveSrcBackup[] =
{
	cv::Point2f(565, 470),
	cv::Point2f(721, 470),
	cv::Point2f(277, 698),
	cv::Point2f(1142, 698)
};

cv::Point2f perspectiveSrc[] = 
{
	cv::Point2f(568, 470),
	cv::Point2f(717, 470),
	cv::Point2f(260, 680),
	cv::Point2f(1043, 680)
};

cv::Point2f perspectiveDst[] =
{
	cv::Point2f(300, 0),
	cv::Point2f(980, 0),
	cv::Point2f(300, 720),
	cv::Point2f(980, 720)
};

int main(int argc, char **argv)
{
	perspectiveMatrix = cv::getPerspectiveTransform(perspectiveSrc, perspectiveDst);

	if (argc < 2)
	{
		std::cerr << "There is no input video." << std::endl;
		return -1;
	}

	videoCapture.open(argv[1]);

	if (!videoCapture.isOpened())
	{
		std::cerr << "Could not open the video." << std::endl;
		return -1;
	}

	videoSize = cv::Size((int)videoCapture.get(cv::CAP_PROP_FRAME_WIDTH),
		(int)videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));

	cv::FileStorage fsRead;

	fsRead.open("Intrinsic.xml", cv::FileStorage::READ);

	if (fsRead.isOpened() == false)
	{
		CameraCalibrator calibrator;
		calibrator.doCalibration(cameraMatrix, dist);
		cv::FileStorage fs;
		fs.open("Intrinsic.xml", cv::FileStorage::WRITE);
		fs << "CameraMatrix" << cameraMatrix;
		fs << "Dist" << dist;
		fs.release();
		fsRead.release();
		std::cout << "There is no existing intrinsic parameters XML file." << std::endl;
		std::cout << "Start calibraton......" << std::endl;
	}
	else
	{
		fsRead["CameraMatrix"] >> cameraMatrix;
		fsRead["Dist"] >> dist;
		fsRead.release();
	}

	float laneDistance = 0;
	std::string text;
	cv::Mat finalResult;

	cv::namedWindow("Real Time Execution", CV_WINDOW_NORMAL);
	cv::namedWindow("Histogram", CV_WINDOW_NORMAL);
	cv::namedWindow("Canny", CV_WINDOW_NORMAL);
	cv::namedWindow("WhiteYellow", CV_WINDOW_NORMAL);
	cv::namedWindow("Warp", CV_WINDOW_NORMAL);

	videoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
	videoCapture >> videoFrame;

	undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
	_videoFrameUndistorted = videoFrameUndistorted.clone();

	LaneDetector detector(_videoFrameUndistorted, perspectiveMatrix);
	cv::undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
	_videoFrameUndistorted = videoFrameUndistorted.clone();

	while (!videoFrame.empty())
	{
		warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

		detector.detectLanes();
		finalResult = detector.getFinalResult();

		laneDistance = detector.getLaneCenterDistance();

		if (laneDistance > 0)
			text = std::to_string(abs(laneDistance)) + "m to the right";
		else
			text = std::to_string(abs(laneDistance)) + "m to the left";

		putText(finalResult, text, cv::Point(50, 50), 0, 2, cv::Scalar(255, 170, 255), 2);

		cv::imshow("Real Time Execution", finalResult);
		cv::imshow("Histogram", detector.getHistogramImage());
		cv::imshow("Canny", detector.getWarpEdgeDetectResult());
		cv::imshow("WhiteYellow", detector.getWhiteYellow());
		cv::imshow("Warp", detector.getWarpImage());

		videoCapture >> videoFrame;

		if (videoFrame.empty())
			break;

		undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
		_videoFrameUndistorted = videoFrameUndistorted.clone();
		detector.setInputImage(_videoFrameUndistorted);

		if (cv::waitKey(15) == 27)
			break;
	}

	return 0;
}
#pragma once

#include <sstream>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

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
	//cv::Point2f(568, 470), //стандарт
	//cv::Point2f(717, 470),
	//cv::Point2f(260, 680),
	//cv::Point2f(1043, 680)

	//cv::Point2f(568, 470), //сдвиг вправо
	//cv::Point2f(767, 470),
	//cv::Point2f(260, 680),
	//cv::Point2f(1143, 680)

	cv::Point2f(548, 470), //сдвиг вправо в2
	cv::Point2f(777, 470),
	cv::Point2f(260, 680),
	cv::Point2f(1053, 680)

	//cv::Point2f(470, 470), //ручное
	//cv::Point2f(853, 470),
	//cv::Point2f(165, 680),
	//cv::Point2f(1278, 680)

	//cv::Point2f(525, 470), //ручное
	//cv::Point2f(864, 470),
	//cv::Point2f(197, 680),
	//cv::Point2f(1278, 680)
};

cv::Point2f perspectiveDst[] =
{
	cv::Point2f(300, 0),
	cv::Point2f(980, 0),
	cv::Point2f(300, 720),
	cv::Point2f(980, 720)
};

int currentDisplay = 0;

LaneDetector* detector;

namespace Capture1 {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Runtime::InteropServices;

	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			InitializeComponent();
		}

	protected:
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}

#pragma region Window_control
	private: System::Windows::Forms::Panel^  panel1;

	private: System::Windows::Forms::Label^  label7;

	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::ComboBox^  cbDisplaySelect;


	private: System::Windows::Forms::TrackBar^  trackBar1;
	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::Windows::Forms::OpenFileDialog^  openFileDialog1;
	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::PictureBox^  pbDisplay;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::PictureBox^  pbHistogram;
	private: System::Windows::Forms::Button^  button1;

	private: System::ComponentModel::IContainer^  components;
#pragma endregion

	protected:

	private:

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->panel1 = (gcnew System::Windows::Forms::Panel());
			this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->pbHistogram = (gcnew System::Windows::Forms::PictureBox());
			this->pbDisplay = (gcnew System::Windows::Forms::PictureBox());
			this->cbDisplaySelect = (gcnew System::Windows::Forms::ComboBox());
			this->openFileDialog1 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->panel1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->groupBox3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbHistogram))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbDisplay))->BeginInit();
			this->SuspendLayout();
			// 
			// panel1
			// 
			this->panel1->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel1->Controls->Add(this->trackBar1);
			this->panel1->Controls->Add(this->pictureBox1);
			this->panel1->Controls->Add(this->label7);
			this->panel1->Controls->Add(this->groupBox3);
			this->panel1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->panel1->Location = System::Drawing::Point(3, 3);
			this->panel1->Name = L"panel1";
			this->panel1->Size = System::Drawing::Size(1259, 584);
			this->panel1->TabIndex = 0;
			// 
			// trackBar1
			// 
			this->trackBar1->AutoSize = false;
			this->trackBar1->Location = System::Drawing::Point(367, 541);
			this->trackBar1->Name = L"trackBar1";
			this->trackBar1->Size = System::Drawing::Size(872, 26);
			this->trackBar1->TabIndex = 1;
			this->trackBar1->Scroll += gcnew System::EventHandler(this, &Form1::trackBar1_Scroll);
			// 
			// pictureBox1
			// 
			this->pictureBox1->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->pictureBox1->Location = System::Drawing::Point(367, 13);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(872, 522);
			this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBox1->TabIndex = 0;
			this->pictureBox1->TabStop = false;
			// 
			// label7
			// 
			this->label7->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->label7->Location = System::Drawing::Point(7, 541);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(165, 25);
			this->label7->TabIndex = 3;
			this->label7->Text = L"Время:";
			this->label7->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->button1);
			this->groupBox3->Controls->Add(this->label1);
			this->groupBox3->Controls->Add(this->button2);
			this->groupBox3->Controls->Add(this->pbHistogram);
			this->groupBox3->Controls->Add(this->pbDisplay);
			this->groupBox3->Controls->Add(this->cbDisplaySelect);
			this->groupBox3->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->groupBox3->Location = System::Drawing::Point(3, 3);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(344, 532);
			this->groupBox3->TabIndex = 10;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Видео";
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(249, 273);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(79, 30);
			this->button1->TabIndex = 3;
			this->button1->Text = L"***";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click);
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(6, 284);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(97, 19);
			this->label1->TabIndex = 2;
			this->label1->Text = L"Гистограмма";
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(249, 22);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(79, 30);
			this->button2->TabIndex = 1;
			this->button2->Text = L"Открыть";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// pbHistogram
			// 
			this->pbHistogram->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->pbHistogram->Location = System::Drawing::Point(8, 306);
			this->pbHistogram->Name = L"pbHistogram";
			this->pbHistogram->Size = System::Drawing::Size(320, 213);
			this->pbHistogram->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pbHistogram->TabIndex = 0;
			this->pbHistogram->TabStop = false;
			// 
			// pbDisplay
			// 
			this->pbDisplay->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->pbDisplay->Location = System::Drawing::Point(8, 56);
			this->pbDisplay->Name = L"pbDisplay";
			this->pbDisplay->Size = System::Drawing::Size(320, 213);
			this->pbDisplay->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pbDisplay->TabIndex = 0;
			this->pbDisplay->TabStop = false;
			this->pbDisplay->DoubleClick += gcnew System::EventHandler(this, &Form1::pbDisplay_DoubleClick);
			// 
			// cbDisplaySelect
			// 
			this->cbDisplaySelect->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->cbDisplaySelect->FormattingEnabled = true;
			this->cbDisplaySelect->Items->AddRange(gcnew cli::array< System::Object^  >(6) {
				L"Оператор Канни", L"Желто-белый фильтр", L"Перспективная проекция",
					L"Обнаружение линий", L"Черно-белая маска", L"Цветокоррекция"
			});
			this->cbDisplaySelect->Location = System::Drawing::Point(8, 23);
			this->cbDisplaySelect->Name = L"cbDisplaySelect";
			this->cbDisplaySelect->Size = System::Drawing::Size(235, 27);
			this->cbDisplaySelect->TabIndex = 1;
			this->cbDisplaySelect->SelectedIndexChanged += gcnew System::EventHandler(this, &Form1::cbDisplaySelect_SelectedIndexChanged);
			this->cbDisplaySelect->TextUpdate += gcnew System::EventHandler(this, &Form1::cbDisplaySelect_TextUpdate);
			this->cbDisplaySelect->SelectedIndex = 0;
			// 
			// openFileDialog1
			// 
			this->openFileDialog1->FileName = L"openFileDialog1";
			// 
			// timer1
			// 
			this->timer1->Interval = 30;
			this->timer1->Tick += gcnew System::EventHandler(this, &Form1::timer1_Tick);
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(10, 23);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1265, 590);
			this->Controls->Add(this->panel1);
			this->Font = (gcnew System::Drawing::Font(L"Calibri", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedDialog;
			this->Margin = System::Windows::Forms::Padding(5);
			this->MaximizeBox = false;
			this->MinimizeBox = false;
			this->Name = L"Form1";
			this->Padding = System::Windows::Forms::Padding(3);
			this->StartPosition = System::Windows::Forms::FormStartPosition::CenterScreen;
			this->Text = L"Распознавание дорожных полос";
			this->panel1->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbHistogram))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbDisplay))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion

	private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e)
	{
		if (button2->Text == "Открыть")
		{
			openFileDialog1->Filter = "AVI files (*.avi)|*.txt|All files (*.*)|*.*";
			openFileDialog1->FilterIndex = 2;
			openFileDialog1->RestoreDirectory = true;
			openFileDialog1->FileName = "";

			if (openFileDialog1->ShowDialog() == System::Windows::Forms::DialogResult::OK)
			{
				perspectiveMatrix = cv::getPerspectiveTransform(perspectiveSrc, perspectiveDst);

				char *fileName = (char*)Marshal::StringToHGlobalAnsi(openFileDialog1->FileName).ToPointer();

				videoCapture.open(fileName);

				videoSize = cv::Size((int)videoCapture.get(cv::CAP_PROP_FRAME_WIDTH),
					(int)videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));

				trackBar1->Minimum = 0;
				trackBar1->Maximum = (int)videoCapture.get(CV_CAP_PROP_FRAME_COUNT);

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
				}
				else
				{
					fsRead["CameraMatrix"] >> cameraMatrix;
					fsRead["Dist"] >> dist;
					fsRead.release();
				}

				videoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
				videoCapture >> videoFrame;

				undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
				_videoFrameUndistorted = videoFrameUndistorted.clone();

				detector = new LaneDetector(_videoFrameUndistorted, perspectiveMatrix);

				button1->Text = "Пауза";
				timer1->Start();

				return;
			}
		}
	}
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e)
	{
		if (button1->Text == "Пауза")
		{
			timer1->Stop();
			button1->Text = "Продолжить";

			return;
		}

		if (button1->Text == "Продолжить")
		{
			button1->Text = "Пауза";
			timer1->Start();

			return;
		}
	}
	private: System::Void trackBar1_Scroll(System::Object^  sender, System::EventArgs^  e)
	{
		videoCapture.set(CV_CAP_PROP_POS_FRAMES, trackBar1->Value);
	}

	private: void DrawCVImage(System::Windows::Forms::Control^ control, cv::Mat& colorImage, System::Drawing::Imaging::PixelFormat f)
	{
		System::Drawing::Graphics^ graphics = control->CreateGraphics();
		System::IntPtr ptr(colorImage.ptr());
		System::Drawing::Bitmap^ b = gcnew System::Drawing::Bitmap(colorImage.cols, colorImage.rows, colorImage.step, f, ptr);
		System::Drawing::RectangleF rect(0, 0, control->Width, control->Height);
		graphics->DrawImage(b, rect);
		delete graphics;
	}

	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e)
	{
		try
		{
			if (!videoFrame.empty())
			{
				float laneDistance = 0;
				std::string text;
				cv::Mat finalResult;
				std::stringstream ss;

				warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

				detector->detectLanes();
				finalResult = detector->getFinalResult();

				laneDistance = detector->getLaneCenterDistance();

				ss.setf(std::ios_base::fixed, std::ios_base::floatfield);
				ss << std::setprecision(3) << abs(laneDistance);

				if (laneDistance > 0)
					text = ss.str() + "m >>>";
				else
					text = "<<< " + ss.str() + "m";

				int baseline = 0;

				std::string ideal = "0.000m";
				cv::Size idealSize = cv::getTextSize(ideal, cv::FONT_HERSHEY_COMPLEX, 2, 2, &baseline);
				cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_COMPLEX, 2, 2, &baseline);

				int textPosX = (finalResult.size().width - idealSize.width) / 2;

				if (laneDistance <= 0)
					textPosX -= (textSize.width - idealSize.width);

				cv::putText(finalResult, text, cv::Point(textPosX,
					finalResult.size().height - 50), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 0, 255), 2);

				DrawCVImage(pictureBox1, finalResult, System::Drawing::Imaging::PixelFormat::Format24bppRgb);

				trackBar1->Value = (int)videoCapture.get(CV_CAP_PROP_POS_FRAMES);
				label7->Text = "Время: " + (TimeSpan::FromMilliseconds(videoCapture.get(CV_CAP_PROP_POS_MSEC)).ToString())->Substring(0, 8);

				DrawCVImage(pbHistogram, detector->getHistogramImage(), System::Drawing::Imaging::PixelFormat::Format24bppRgb);
				
				switch (currentDisplay)
				{
				case 0:
					DrawCVImage(pbDisplay, detector->getWarpEdgeDetectResult(), System::Drawing::Imaging::PixelFormat::Format8bppIndexed);
					break;
				case 1:
					DrawCVImage(pbDisplay, detector->getWhiteYellow(), System::Drawing::Imaging::PixelFormat::Format24bppRgb);
					break;
				case 2:
					DrawCVImage(pbDisplay, detector->getWarpImage(), System::Drawing::Imaging::PixelFormat::Format24bppRgb);
					break;
				case 3:
					DrawCVImage(pbDisplay, detector->getMergeImage(), System::Drawing::Imaging::PixelFormat::Format24bppRgb);
					break;
				case 4:
					DrawCVImage(pbDisplay, detector->getMaskImage(), System::Drawing::Imaging::PixelFormat::Format24bppRgb);
					break;
				case 5:
					DrawCVImage(pbDisplay, detector->getHSL(), System::Drawing::Imaging::PixelFormat::Format24bppRgb);
					break;
				}

				videoCapture >> videoFrame;

				if (!videoFrame.empty())
				{
					undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
					_videoFrameUndistorted = videoFrameUndistorted.clone();
					detector->setInputImage(_videoFrameUndistorted);
				}
			}
		}
		catch (...) {}
	}
	private: System::Void cbDisplaySelect_TextUpdate(System::Object^  sender, System::EventArgs^  e)
	{

	}
private: System::Void pbDisplay_DoubleClick(System::Object^  sender, System::EventArgs^  e) 
{
	currentDisplay++;

	if (currentDisplay > 4)
		currentDisplay = 0;
}
private: System::Void cbDisplaySelect_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) 
{
	if (cbDisplaySelect->Text == "Оператор Канни")
		currentDisplay = 0;
	if (cbDisplaySelect->Text == "Желто-белый фильтр")
		currentDisplay = 1;
	if (cbDisplaySelect->Text == "Перспективная проекция")
		currentDisplay = 2;
	if (cbDisplaySelect->Text == "Обнаружение линий")
		currentDisplay = 3;
	if (cbDisplaySelect->Text == "Черно-белая маска")
		currentDisplay = 4;
	if (cbDisplaySelect->Text == "Цветокоррекция")
		currentDisplay = 5;
}
};
}


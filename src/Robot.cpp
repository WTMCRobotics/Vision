#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <vision/VisionPipeline.h>
#include <CameraServer.h>
#include "Grip.h"

#include <XboxController.h>
#include <TankDrive.h>

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		camera = frc::CameraServer::GetInstance()->StartAutomaticCapture("Camera", 0);
		outputSource = frc::CameraServer::GetInstance()->PutVideo("video", 640, 480);
		cvSink = frc::CameraServer::GetInstance()->GetVideo();

		leftStick = 0;
		rightStick = 0;
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		cvSink.GrabFrame(image);
		gripObj.process(image);

		contourPointsPtr = gripObj.getfilterContoursOutput();
		contourPoints = *contourPointsPtr;

		std::cout <<contourPoints.size() << std::endl;
		if(contourPoints.size() > 0)
		{
			rectangle = cv::boundingRect(contourPoints[0]);

			largestArea = rectangle.area();
			largestIndex = 0;
			for (unsigned int i = 1; i < contourPoints.size(); i++)
			{
				std::cout<<"yes\n";
				rectangle = cv::boundingRect(contourPoints[i]);
				contourArea = rectangle.area();
				if(contourArea > largestArea)
				{
					largestArea = contourArea;
					largestIndex = i;
				}
			}
			rectangle = cv::boundingRect(contourPoints[largestIndex]);
			centerX = rectangle.x + (rectangle.width / 2);
			centerY = rectangle.y + (rectangle.height / 2);

			frc::SmartDashboard::PutNumber("Center X", centerX);
			frc::SmartDashboard::PutNumber("Center Y", centerY);
			cv::drawContours(image, contourPoints, largestIndex, cv::Scalar(255,0,0), 2);
			cv::drawMarker(image, cv::Point2i(centerX,centerY), cv::Scalar(255,0,0), cv::MARKER_DIAMOND, 2);
			cv::drawMarker(image, cv::Point2i(image.size().width/2, image.size().height/2), cv::Scalar(0,255,0), cv::MARKER_STAR, 2);
			outputSource.PutFrame(image);
		}
		else
		{
			frc::SmartDashboard::PutNumber("Center X", -1);
			frc::SmartDashboard::PutNumber("Center Y", -1);
		}
		drivetrain.DriveVision(centerX, centerY, image.size().width);
	}

	void TeleopInit() {
		drivetrain.Stop();

	}

	void TeleopPeriodic() {
		cvSink.GrabFrame(image);
		gripObj.process(image);

		contourPointsPtr = gripObj.getfilterContoursOutput();
		contourPoints = *contourPointsPtr;

		std::cout <<contourPoints.size() << std::endl;
		if(contourPoints.size() > 0)
		{
			rectangle = cv::boundingRect(contourPoints[0]);

			largestArea = rectangle.area();
			largestIndex = 0;
			for (unsigned int i = 1; i < contourPoints.size(); i++)
			{
				std::cout<<"yes\n";
				rectangle = cv::boundingRect(contourPoints[i]);
				contourArea = rectangle.area();
				if(contourArea > largestArea)
				{
					largestArea = contourArea;
					largestIndex = i;
				}
			}
			rectangle = cv::boundingRect(contourPoints[largestIndex]);
			centerX = rectangle.x + (rectangle.width / 2);
			centerY = rectangle.y + (rectangle.height / 2);

			frc::SmartDashboard::PutNumber("Center X", centerX);
			frc::SmartDashboard::PutNumber("Center Y", centerY);
			cv::drawContours(image, contourPoints, largestIndex, cv::Scalar(255,0,0), 2);
			cv::drawMarker(image, cv::Point2i(centerX,centerY), cv::Scalar(255,0,0), cv::MARKER_DIAMOND, 2);
			cv::drawMarker(image, cv::Point2i(image.size().width/2, image.size().height/2), cv::Scalar(0,255,0), cv::MARKER_STAR, 2);
			outputSource.PutFrame(image);
		}
		else
		{
			frc::SmartDashboard::PutNumber("Center X", -1);
			frc::SmartDashboard::PutNumber("Center Y", -1);
		}

		//UpdateControls();
		//drivetrain.Drive(leftStick, rightStick);
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	double leftStick;
	double rightStick;

	cs::UsbCamera camera;
	cs::CvSink cvSink;

	cv::Rect rectangle;
	std::vector<std::vector<cv::Point>>* contourPointsPtr;
	std::vector<std::vector<cv::Point>> contourPoints;
	cv::Mat image;
	cs::CvSource outputSource;
	grip::Grip gripObj;
	int contourArea;
	int largestIndex;
	int largestArea;
	int centerX;
	int centerY;

	frc::XboxController xbox{0};
	TankDrive drivetrain;


	void UpdateControls()
	{
		leftStick = xbox.GetY(GenericHID::kLeftHand);
		rightStick = xbox.GetY(GenericHID::kRightHand);
	}
};

START_ROBOT_CLASS(Robot)

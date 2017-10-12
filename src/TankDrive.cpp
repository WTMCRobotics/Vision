/*
 * TankDrive.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: classroom
 */

#include <GenericHID.h>
#include <SPI.h>
#include <TankDrive.h>
#include <GlobalVariables.h>

TankDrive::TankDrive() : left2(0), right2(0)
{
	leftMaster.SetControlMode(CANSpeedController::kSpeed);
	leftMaster.SetFeedbackDevice(CANTalon::QuadEncoder);
	leftMaster.ConfigEncoderCodesPerRev(2048);
	leftMaster.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	leftMaster.SetVoltageRampRate(24.0);
	leftMaster.SetSensorDirection(true);
	leftMaster.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);

	rightMaster.SetControlMode(CANSpeedController::kSpeed);
	rightMaster.SetFeedbackDevice(CANTalon::QuadEncoder);
	rightMaster.ConfigEncoderCodesPerRev(2048);
	rightMaster.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	rightMaster.SetVoltageRampRate(24.0);
	rightMaster.SetSensorDirection(true);
	rightMaster.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);

	leftSlave.SetControlMode(CANSpeedController::kFollower);
	leftSlave.Set(LEFT_MASTER);

	rightSlave.SetControlMode(CANSpeedController::kFollower);
	rightSlave.Set(RIGHT_MASTER);
}


void TankDrive::Stop()
{
	leftMaster.Set(0);
	rightMaster.Set(0);
}

void TankDrive::Drive(double left, double right)
{
	left2 = (abs(left) > DEADBAND) ? left * MAX_SPEED : 0;
	right2 = (abs(right) > DEADBAND) ? right * MAX_SPEED : 0;

	leftMaster.Set(-left2);
	rightMaster.Set(right2);
}

/*
 * TankDrive.h
 *
 *  Created on: Oct 5, 2017
 *      Author: classroom
 */

#ifndef SRC_TANKDRIVE_H_
#define SRC_TANKDRIVE_H_

#include <CANTalon.h>
#include <XboxController.h>
#include <math.h>
#include <GlobalVariables.h>
#include <CameraServer.h>

class TankDrive
{

	public:
		TankDrive();
		void Stop();
		void Drive(double left, double right);
		void DriveVision(double xCenter, double yCenter, double width);
	private:
		CANTalon leftMaster {LEFT_MASTER};
		CANTalon leftSlave {LEFT_SLAVE};
		CANTalon rightMaster {RIGHT_MASTER};
		CANTalon rightSlave {RIGHT_SLAVE};

		double left2;
		double right2;
};

#endif /* SRC_TANKDRIVE_H_ */

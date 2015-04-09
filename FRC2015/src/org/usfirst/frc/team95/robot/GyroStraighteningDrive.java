package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;


public class GyroStraighteningDrive{
	
	Gyro gyro;
	RobotDrive drive;

	public GyroStraighteningDrive(RobotDrive driveTrain, Gyro gyroscope) {
		drive = driveTrain;
		gyro = gyroscope;
	}
	
	public void mecanumDrive_Cartesian(double x, double y, double rotate, double heading) {
		drive.mecanumDrive_Cartesian(x, y, rotate + gyro.getAngle() / 100, heading);
	}

}

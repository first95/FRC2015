package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.ResetableGyro;
import org.usfirst.frc.team95.robot.Robot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class GoExactlyForward extends AutoMove {
	RobotDrive drive;
	ResetableGyro gyro;
	Timer timeOut;
	double timeLimit;
	
	public GoExactlyForward(Robot robot, double time) {
		drive = robot.driveTrain;
		gyro = robot.gyro;
		timeOut = new Timer();
		timeLimit = time;
	}
	
	public Status init() {
		gyro.reset();
		timeOut.reset();
		timeOut.start();
		return Status.wantsToContinue;
	}
	
	public Status periodic() {
		if (timeOut.get() > timeLimit) {
			return Status.isAbleToContinue;
		} else {
			drive.mecanumDrive_Cartesian(0, 0.75, gyro.getAngle() / 100, 2);
		}
		return Status.wantsToContinue;
	}
	
	public Status stop() {
		return Status.isNotAbleToContinue;
	}

}

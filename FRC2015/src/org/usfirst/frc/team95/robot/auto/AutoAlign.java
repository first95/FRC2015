package org.usfirst.frc.team95.robot.auto;

/**
 * Move in a certain way for a period of time.
 */

import org.usfirst.frc.team95.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class AutoAlign extends AutoMove {
	public double t;
	Timer timeOut;
	Robot robot;

	public AutoAlign(Robot robo, double maxAlignTime) {
		t = maxAlignTime;
		timeOut = new Timer();
		timeOut.reset();
		robot = robo;
	}

	public Status init() {
		timeOut.start();
		System.out.println("I was started.");
		return Status.needsToContinue;
	}

	public Status periodic() {
		System.out.println("Auto aligning . . . ");
		if (timeOut.get() >= t) {
			return stop();
		} else {
			double x = 0, y = 0, r = 0;
			double[] newDriveValues = robot.autoAlign(0, 0, 0, false);
			x = newDriveValues[0];
			y = newDriveValues[1];
//			r = newDriveValues[2]; // this had some issues, so restricting movement to just x and y
		
			robot.driveTrain.mecanumDrive_Cartesian(x, y, r, 0);
			return Status.wantsToContinue;
		}
	}

	public Status stop() {
		return Status.isNotAbleToContinue;
	}

}

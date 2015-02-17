package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

/**
 * Stay still.
 * 
 * @author daroc
 * 
 */

public class NoMove extends AutoMove {

	Robot robot;

	public NoMove(Robot robo) {
		robot = robo;
	}

	public Status init() {
		return Status.wantsToContinue;
	}

	public Status periodic() {
		robot.driveTrain.mecanumDrive_Cartesian(0, 0, 0, 0);
		return Status.wantsToContinue;
	}

	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

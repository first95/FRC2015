package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;

/**
 * Picks up a tote from the ground.
 * 
 * @author daroc
 * 
 */

public class PickUpTote extends AutoMove {

	Robot robot;
	FauxPID fingerController;

	public PickUpTote(Robot robo) {
		robot = robo;
		fingerController = robo.fingerController;
	}

	public Status init() {
		robot.startedMovingTimeOut.reset();
		robot.startedMovingTimeOut.start();
		/*System.out.print("Pick tote initialization.");
		fingerController.setSetpoint(findSetpoint(fingerController
				.getSetpoint()) + 1);*/
		return Status.wantsToContinue;
	}

	public Status periodic() {
		if ((!(robot.topFingerLimitSwitch.get() && robot.lowFingerLimitSwitch.get() && 
				robot.midLowFingerLimitSwitch.get() && robot.midHighFingerLimitSwitch.get())) 
				&& robot.startedMovingTimeOut.get() > 0.2) {
			robot.realFingerMotor.set(0);
			return Status.isNotAbleToContinue;
		} else {
			robot.realFingerMotor.set(0.5);
			return Status.wantsToContinue;
		}
		/*System.out.println("Pick Tote");
		if (fingerController.onTarget()) {
			return Status.isNotAbleToContinue;
		} else {
			return Status.needsToContinue;
		}*/
	}

	public Status stop() {
		robot.realFingerMotor.set(0);
		return Status.isNotAbleToContinue;
	}

	private int findSetpoint(double point) {
		int index = -1;
		for (int i = 0; i < RobotConstants.kFingerSetpoints.length; i++) {
			if (Math.abs(point - RobotConstants.kFingerSetpoints[i]) < 0.01) {
				index = i;
			}
		}
		return index;
	}

}

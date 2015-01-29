package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;

public class PickUpCan extends AutoMove {
	Robot robot;

	public PickUpCan(Robot robo) {
		robot = robo;
	}

	@Override
	public Status init() {
		robot.armController.setSetpoint(RobotConstants.kArmPositionGrab);
		return Status.wantsToContinue;
	}

	@Override
	public Status periodic() {
		if (robot.armController.onTarget() && 
				robot.armController.getSetpoint() == RobotConstants.kArmPositionGrab) {
			robot.armPistons.set(true);
			robot.armController.setSetpoint(RobotConstants.kArmPositionZenith);
			return Status.needsToContinue;
		} else if (robot.armController.onTarget() && 
				robot.armController.getSetpoint() == RobotConstants.kArmPositionZenith) {
			return Status.isNotAbleToContinue;
		} else {
			return Status.needsToContinue;
		}
	}

	@Override
	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

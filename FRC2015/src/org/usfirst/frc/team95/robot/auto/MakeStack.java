package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;
import edu.wpi.first.wpilibj.Timer;

public class MakeStack extends AutoMove {
	Robot robot;
	Timer timeOut;
	int stage = 0;

	public MakeStack(Robot robo) {
		robot = robo;
	}

	@Override
	public Status init() {
		robot.fingerController.setSetpoint(RobotConstants.kFingerSetpoints[0]);
		stage = 0;
		timeOut = new Timer();
		timeOut.stop();
		timeOut.reset();
		return Status.needsToContinue;
	}

	@Override
	public Status periodic() {
		if (robot.fingerController.onTarget() && stage == 0) {
			robot.armController.setSetpoint(RobotConstants.kArmPositionGrab);
			timeOut.start();
			stage = 1;
		} else if (stage == 1 && (timeOut.get() > 2)) {
			robot.armPistons.set(false);
			robot.armController.setSetpoint(RobotConstants.kArmPositionZenith);
			stage = 2;
		} else if (stage == 2) {
			return Status.isAbleToContinue;
		}

		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		// TODO Auto-generated method stub
		return null;
	}
}

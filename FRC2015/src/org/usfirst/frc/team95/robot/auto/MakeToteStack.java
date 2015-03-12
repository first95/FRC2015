package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;

import edu.wpi.first.wpilibj.Timer;

public class MakeToteStack extends AutoMove {
	Robot robot;
	Timer timeOut;

	public MakeToteStack(Robot robo) {
		robot = robo;
	}

	@Override
	public Status init() {
		robot.fingerController.setSetpoint(RobotConstants.kFingerSetpoints[0]);
		timeOut = new Timer();
		timeOut.stop();
		timeOut.reset();
		return Status.needsToContinue;
	}

	@Override
	public Status periodic() {
		if (robot.fingerController.onTarget() || timeOut.get() > 2) {
			return Status.isAbleToContinue;
		}
		
		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

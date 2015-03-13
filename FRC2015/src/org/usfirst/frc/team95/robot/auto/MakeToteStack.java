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
		robot.realFingerMotor.set(-0.5);
		return Status.needsToContinue;
	}

	@Override
	public Status periodic() {
		if (!robot.lowFingerLimitSwitch.get()) {
			robot.realFingerMotor.set(0);
			return Status.isAbleToContinue;
		}
		robot.realFingerMotor.set(-0.5);
		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

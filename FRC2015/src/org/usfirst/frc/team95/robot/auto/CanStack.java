package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;
import edu.wpi.first.wpilibj.Timer;

public class CanStack extends AutoMove {
	Robot robot;
	int stage = 0;

	public CanStack(Robot robo) {
		robot = robo;
	}

	@Override
	public Status init() {
		return null;
	}

	@Override
	public Status periodic() {
		return null;
	}

	@Override
	public Status stop() {
		// TODO Auto-generated method stub
		return null;
	}
}

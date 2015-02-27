package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class TimedArmMove extends AutoMove {
	Timer timeOut;
	Robot robo;
	double s, t;

	public TimedArmMove(Robot robot, double speed, double time) {
		s = speed;
		t = time;
		robo = robot;
		timeOut = new Timer();
		timeOut.start();
	}

	@Override
	public Status init() {
		timeOut.reset();
		timeOut.start();
		robo.armController.setSetpoint(s);
		return Status.wantsToContinue;
	}

	@Override
	public Status periodic() {
		if (timeOut.get() > t) {
			robo.armController.setSetpoint(0);
			return Status.isNotAbleToContinue;
		}
		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		// TODO Auto-generated method stub
		return null;
	}

}

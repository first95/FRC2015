package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class MoveRelative extends AutoMove {
	public double x, y, r, t;
	Timer timeOut;
	Robot robot;
	
	public MoveRelative(Robot robo, double xMove, double yMove, double rotate, double time) {
		x = xMove;
		y = yMove;
		r = rotate;
		t = time;
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
		if (timeOut.get() >= t) {
			return stop();
		} else {
			robot.driveTrain.mecanumDrive_Cartesian(x, y, r, 0);
			return Status.wantsToContinue;
		}
	}
	
	public Status stop() {
		return Status.isNotAbleToContinue;
	}

}

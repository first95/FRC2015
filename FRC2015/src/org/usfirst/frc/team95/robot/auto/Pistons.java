package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class Pistons extends AutoMove {
	
	Robot robot;
	boolean on;
	
	public Pistons(Robot robo, boolean b) {
		robot = robo;
		on = b;
	}
	
	public Status init() {
		return Status.wantsToContinue;
	}
	
	public Status periodic() {
		robot.armPistons.set(on);
		return Status.isNotAbleToContinue;
	}
	
	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

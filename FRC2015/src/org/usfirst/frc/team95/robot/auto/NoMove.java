package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;


public class NoMove extends AutoMove {

	Robot robot;
	
	public NoMove(Robot robo) {
		robot = robo;
	}
	
	public Status init() {
		return Status.isAbleToContinue;
	}
	
	public Status periodic() {
		return Status.isNotAbleToContinue;
	}
	
	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

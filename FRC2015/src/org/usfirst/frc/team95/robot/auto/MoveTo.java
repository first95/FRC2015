package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class MoveTo extends AutoMove {

	Robot robot;
	double targetX, targetY;
	
	public MoveTo(Robot robo, double x, double y) {
		robot = robo;
		targetX = x;
		targetY = y;
		
	}
	
	public Status init() {
		return Status.wantsToContinue;
	}
	
	
}

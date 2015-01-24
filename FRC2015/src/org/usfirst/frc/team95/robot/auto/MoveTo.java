package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.PIDMecanumOutput;
import org.usfirst.frc.team95.robot.Robot;

public class MoveTo extends AutoMove {

	Robot robot;
	PIDMecanumOutput xPID, yPID, rotationPID;
	double targetX, targetY, targetRotation;
	
	public MoveTo(Robot robo, double x, double y, double rotation) {
		robot = robo;
		targetX = x;
		targetY = y;
		targetRotation = rotation;
	}
	
	public Status init() {
		return Status.wantsToContinue;
	}
	
	public Status periodic() {
		
	}
	
	public Status stop() {
		
	}
	
}

package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class MoveArmTo extends AutoMove{
	Robot robot;
	public double target;
	public MoveArmTo(Robot robo, double target){
		this.target = target;
		robot = robo;
	}
	
	public Status init() {
		robot.armController.setSetpoint(target);
		return Status.needsToContinue;
	}
	
	public Status periodic(){
		if (robot.armController.onTarget()) {
			return Status.isNotAbleToContinue;
		}
		else {
			return Status.wantsToContinue;
		}
	}
	
	public Status stop(){
		return Status.isNotAbleToContinue;
	}
}

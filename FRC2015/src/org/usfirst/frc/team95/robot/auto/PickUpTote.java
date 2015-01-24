package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.auto.AutoMove.Status;

public class PickUpTote extends AutoMove {
	
	Robot robot;
	
	public PickUpTote(Robot robo){
		robot = robo;
	}
	
	public Status init() {
		return Status.wantsToContinue;
	}
}

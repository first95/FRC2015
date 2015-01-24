package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import edu.wpi.first.wpilibj.PIDController;

public class PickUpTote extends AutoMove {
	
	Robot robot;
	PIDController fingerController;
	
	public PickUpTote(Robot robo, PIDController fingers){
		robot = robo;
		fingerController = robo.fingerController;
	}
	
	public Status init() {
		fingerController.setSetpoint(fingerController.get() + 1);
		return Status.wantsToContinue;
	}

	public Status periodic() {
		if (fingerController.onTarget()) {
			return Status.isNotAbleToContinue;
		}
		else {
			return Status.needsToContinue;
		}
	}

	public Status stop() {
		return Status.isNotAbleToContinue;
	}
		
}

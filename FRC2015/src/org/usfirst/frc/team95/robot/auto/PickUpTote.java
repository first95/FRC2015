package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;

import edu.wpi.first.wpilibj.PIDController;

public class PickUpTote extends AutoMove {
	
	Robot robot;
	PIDController fingerController;
	
	public PickUpTote(Robot robo, PIDController fingers){
		robot = robo;
		fingerController = robo.fingerController;
	}
	
	public Status init() {
		System.out.print("Pick tote initialization.");
		fingerController.setSetpoint(findSetpoint(fingerController.get()) + 1);
		return Status.wantsToContinue;
	}

	public Status periodic() {
		System.out.println("Pick Tote");
		if (true || fingerController.onTarget()) {
			return Status.isNotAbleToContinue;
		}
		else {
			return Status.needsToContinue;
		}
	}

	public Status stop() {
		return Status.isNotAbleToContinue;
	}
	
	private int findSetpoint(double point) {
		int index = -1;
		for (int i = 0; i < RobotConstants.kFingerSetpoints.length; i++) {
			if (Math.abs(point - RobotConstants.kFingerSetpoints[i]) < 0.01) {
				index = i;
			}
		}
		return index;
	}
		
}

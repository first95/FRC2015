package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class MoveArmTo extends AutoMove{
	Robot robot;
	public double target;
	Timer timeOut;
	
	public MoveArmTo(Robot robo, double targe){ // 1.497
		target = targe;
		robot = robo;
		timeOut = new Timer();
		timeOut.reset();
		timeOut.start();
	}
	
	public Status init() {
		if (target < robot.armEncoder.getDistance()) {
			System.out.println("Decided that target was behind us.");
			robot.armMotors.set(0.5);
		} else {
			System.out.println("Decided that target was in front.");
			robot.armMotors.set(-0.5);
		}
		robot.armController.enabled = false;
		timeOut.reset();
		timeOut.start();
		return Status.needsToContinue;
	}
	
	public Status periodic(){
		if ((robot.armEncoder.getDistance() > target && target > 0) ||
				(robot.armEncoder.getDistance() < target && target < 0) || timeOut.get() > 2) {
			System.out.println("ArmTo: I believe myself to be done.");
			
			robot.armMotors.set(0);
			return Status.isNotAbleToContinue;
		} else {
			robot.armMotors.set(-0.5);
		}
		System.out.println("ArmTo: Continuing . . . ");
		return Status.wantsToContinue;
	}
	
	public Status stop(){
		robot.armMotors.set(0);
		robot.armController.enabled = true;
		return Status.isNotAbleToContinue;
	}
}

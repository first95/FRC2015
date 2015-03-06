package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Pistons extends AutoMove {

	Solenoid pistons;
	boolean on;

	public Pistons(Robot robo, boolean b) {
		pistons = robo.armPistons;
		on = b;
	}
	
	public Pistons(Solenoid solenoid, boolean b) {
		pistons = solenoid;
		on = b;
	}

	public Status init() {
		return Status.wantsToContinue;
	}

	public Status periodic() {
		pistons.set(on);
		return Status.isNotAbleToContinue;
	}

	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

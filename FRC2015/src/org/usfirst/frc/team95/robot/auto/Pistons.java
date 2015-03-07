package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pistons extends AutoMove {

	DoubleSolenoid pistons;
	boolean on;

	public Pistons(Robot robo, boolean b) {
		pistons = robo.armPistons;
		on = b;
	}
	
	public Pistons(DoubleSolenoid solenoid, boolean b) {
		pistons = solenoid;
		on = b;
	}

	public Status init() {
		return Status.wantsToContinue;
	}

	public Status periodic() {
		if (on) {
			pistons.set(Value.kForward);
		} else {
			pistons.set(Value.kReverse);
		}
		return Status.isNotAbleToContinue;
	}

	public Status stop() {
		return Status.isNotAbleToContinue;
	}
}

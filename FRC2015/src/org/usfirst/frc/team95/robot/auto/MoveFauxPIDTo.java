package org.usfirst.frc.team95.robot.auto;

public class MoveFauxPIDTo extends AutoMove {
	FauxPID pid;
	double setpoint;
	
	public MoveFauxPIDTo(FauxPID pid, double setpoint) {
		this.pid = pid;
		this.setpoint = setpoint;
	}

	@Override
	public Status init() {
		pid.setSetpoint(setpoint);
		pid.enabled = true;
		return Status.wantsToContinue;
	}

	@Override
	public Status periodic() {
		if (pid.onTarget()) {
			return Status.isAbleToContinue;
		}
		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		return Status.isNotAbleToContinue;
	}

}

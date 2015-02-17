package org.usfirst.frc.team95.robot.auto;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class PlainMotorMove extends AutoMove {
	public double speed, time;
	Timer timeOut;
	SpeedController motor;
	
	public PlainMotorMove(SpeedController motor, double speed, double time) {
		this.time = time;
		this.speed = speed;
		timeOut = new Timer();
		timeOut.reset();
		this.motor = motor;
	}

	@Override
	public Status init() {
		timeOut.start();
		return Status.wantsToContinue;
	}

	@Override
	public Status periodic() {
		if (timeOut.get() > time) {
			return stop();
		} else {
			motor.set(speed);
			return Status.wantsToContinue;
		}
	}

	@Override
	public Status stop() {
		timeOut.stop();
		timeOut.reset();
		return Status.isNotAbleToContinue;
	}

}

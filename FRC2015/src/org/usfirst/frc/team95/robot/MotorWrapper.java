package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.SpeedController;

public class MotorWrapper {
	SpeedController wrappedMotor;
	
	public MotorWrapper(SpeedController wrappedMotor) {
		this.wrappedMotor = wrappedMotor;
	}
	
	public void set(double speed) {
		double currentSpeed;
		double targetSpeed;
		currentSpeed = wrappedMotor.get();
		if (Math.abs(speed - currentSpeed) >= .5) {
			targetSpeed = 
		}
	}

}

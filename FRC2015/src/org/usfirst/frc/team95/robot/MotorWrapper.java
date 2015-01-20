package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.SpeedController;

public class MotorWrapper implements SpeedController {
	SpeedController wrappedMotor;
	
	public MotorWrapper(SpeedController wrappedMotor) {
		this.wrappedMotor = wrappedMotor;
	}
	
	public void set(double speed) {
		double currentSpeed;
		currentSpeed = wrappedMotor.get();
		if (Math.abs(speed - currentSpeed) >= .5) {
			if (speed > 0) {
				wrappedMotor.set(currentSpeed + 0.5);
			} else {
				wrappedMotor.set(currentSpeed + 0.5);
			}
		} else {
			wrappedMotor.set(speed);
		}
	}

	@Override
	public void pidWrite(double output) {
		wrappedMotor.pidWrite(output);
		System.out.println("If this is being printed, the pidWrite of MotorWrapper is being misused.");
		
	}

	@Override
	public double get() {
		return wrappedMotor.get();
	}

	@Override
	public void set(double speed, byte syncGroup) {
		set(speed);
		
	}

	@Override
	public void disable() {
		wrappedMotor.disable();
		
	}

}

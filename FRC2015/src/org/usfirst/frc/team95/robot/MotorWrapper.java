package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

public class MotorWrapper implements SpeedController, PIDOutput {
	SpeedController wrappedMotor;
	public double scaling = 1.0;

	public MotorWrapper(SpeedController wrappedMotor) {
		this.wrappedMotor = wrappedMotor;
	}

	public void set(double speed) {
		speed = speed * scaling;
		double currentSpeed;
		currentSpeed = wrappedMotor.get();
		if (Math.abs(speed - currentSpeed) >= RobotConstants.kMotorSpeedChangeMaximum) {
			if ((speed - currentSpeed) > 0) {
				wrappedMotor.set(currentSpeed
						+ RobotConstants.kMotorSpeedChangeMaximum);
			} else {
				wrappedMotor.set(currentSpeed
						- RobotConstants.kMotorSpeedChangeMaximum);
			}
		} else {
			wrappedMotor.set(speed);
		}
	}

	@Override
	public void pidWrite(double output) {
		wrappedMotor.pidWrite(output);
		// System.out.println("If this is being printed, the pidWrite of MotorWrapper is being misused.");

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

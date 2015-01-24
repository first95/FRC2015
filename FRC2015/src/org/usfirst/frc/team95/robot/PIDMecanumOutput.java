package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.PIDOutput;


public class PIDMecanumOutput implements PIDOutput {
	MotorWrapper frontLeft, frontRight, backLeft, backRight;
	int aspectToControl;
	
	public PIDMecanumOutput(MotorWrapper frontLeft, MotorWrapper frontRight, MotorWrapper backLeft, 
			MotorWrapper backRight, int aspectToControl) {
		this.frontLeft = frontLeft;
		this.frontRight = frontRight;
		this.backLeft = backLeft;
		this.backRight = backRight;
		this.aspectToControl = aspectToControl;
	}

	@Override
	public void pidWrite(double output) {
		double[] speeds = reverse();
		double x = speeds[0];
		double y = speeds[1];
		double r = speeds[2];
		if (aspectToControl == 0) {
			x = output;
		} else if (aspectToControl == 1) {
			y = output;
		} else if (aspectToControl == 2) {
			r = output;
		}
		
		frontLeft.set(x + y + r);
		frontRight.set(-x + y - r);
		backLeft.set(-x + y + r);
		backRight.set(x + y - r);
		
	}
	
	private double[] reverse() {
		double r = (backLeft.get() - frontRight.get()) / 2;
		double x = (backRight.get() - frontRight.get()) / 2;
		double y = x + r + frontRight.get();
		double[] tr = {x, y, r};
		return tr;
	}

}

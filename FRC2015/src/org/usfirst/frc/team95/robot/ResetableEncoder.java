package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.Encoder;

public class ResetableEncoder extends Encoder {
	double offset = 0;

	public ResetableEncoder(int m, int n) {
		super(m, n);
	}

	public void setOffset(double r) {
		offset = r;
	}

	@Override
	public double getDistance() {
		return offset + super.getDistance();
	}

	public void setPosition(double offset) {
		setOffset(-super.getDistance() + offset);
	}
}

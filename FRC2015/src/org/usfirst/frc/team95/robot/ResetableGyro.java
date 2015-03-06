package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Gyro;

public class ResetableGyro extends Gyro {
	double offset, rateOffset;

	public ResetableGyro(AnalogInput channel) {
		super(channel);
		offset = 0;
		rateOffset = 0;
		// TODO Auto-generated constructor stub
	}

	public ResetableGyro(int channel) {
		super(channel);
		offset = 0;
		rateOffset = 0;
	}

	public void set(double thatway) {
		offset = thatway + super.getAngle();
		System.out.println("thatway" + thatway + "\n get anlge"
				+ super.getAngle());

	}

	public void resetRate() {
		offset = -super.getRate();
	}

	public double getAngle() {
		return super.getAngle() + offset;
	}

	public double getRate() {
		return super.getRate() + rateOffset;
	}

}

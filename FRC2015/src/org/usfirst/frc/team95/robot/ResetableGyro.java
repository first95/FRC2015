package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Gyro;

public class ResetableGyro extends Gyro {
	double offset;

	public ResetableGyro(AnalogInput channel) {
		super(channel);
		offset = 0;
		// TODO Auto-generated constructor stub
	}

	public ResetableGyro(int channel) {
		super(channel);
		offset = 0;
	}

	public void set(double thatway) {
		offset = thatway + super.getAngle();
		System.out.println("thatway" + thatway + "\n get anlge" + super.getAngle());
		
	}

	public double getAngle() {
		return super.getAngle() + offset;
	}

}

package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class CommonFilter {
	ADXL345_I2C extra;
	BuiltInAccelerometer builtin;
	
	public CommonFilter(ADXL345_I2C accel1, BuiltInAccelerometer accel2) {
		extra = accel1;
		builtin = accel2;
	}
	
	public double getXAcceleration() {
		return extra.getAcceleration(ADXL345_I2C.Axes.kX) + builtin.getX() / 2;
	}
	
	public double getYAcceleration() {
		return extra.getAcceleration(ADXL345_I2C.Axes.kY) + builtin.getY() / 2;
	}

}

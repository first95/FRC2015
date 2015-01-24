package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.PIDSource;

public class PIDAccelerometerInput implements PIDSource {
	CommonFilter accel;
	int axis;
	
	public PIDAccelerometerInput(CommonFilter accel, int axis) {
		this.accel = accel;
		this.axis = axis;
	}

	@Override
	public double pidGet() {
		if (axis == 0) {
			return accel.getXAcceleration();
		} else if (axis == 1) {
			return accel.getYAcceleration();
		} else if (axis == 2) {
			
		}
	}
}

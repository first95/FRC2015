package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Gyro;

public class ResetableGyro extends Gyro {
	

	public ResetableGyro(AnalogInput channel) {
		super(channel);
		// TODO Auto-generated constructor stub
	}
	
	public ResetableGyro(int channel) {
		super(channel);
	}

}

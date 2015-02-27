package org.usfirst.frc.team95.robot.auto;

import edu.wpi.first.wpilibj.DigitalInput;

public class PureInterruptedSequentialMove extends PureSequentialMove {
	DigitalInput interruptor;
	
	@Override
	public Status periodic() {
		if (interruptor != null && !interruptor.get()) {
			return super.stop();
		}
		return super.periodic();
	}
}

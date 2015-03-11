package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RotateGrabber extends PureSequentialMove{
	
	Robot robot;
	public DoubleSolenoid grabberRotatePiston;
			
	public RotateGrabber(Robot robo){
		grabberRotatePiston.set(Value.kForward);

	}

}

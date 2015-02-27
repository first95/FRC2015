package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabCanFromFloor extends PureSequentialMove {
	
	public GrabCanFromFloor(Robot robo) {
		AutoMove[] stuff = {new Pistons(robo, true), new PlainMotorMove(robo.armMotors, -0.5, 1.5),
				new GoBackward(robo), new PlainMotorMove(robo.armMotors, 0.5, 1.5) };
		sequence = new SequentialMove(stuff);
		
	}
}

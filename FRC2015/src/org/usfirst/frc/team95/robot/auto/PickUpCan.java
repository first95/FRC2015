package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

/**
 * Pick up a can from the set grabbing position.
 * 
 * @author daroc
 * 
 */

public class PickUpCan extends PureSequentialMove {
	Robot robot;

	public PickUpCan(Robot robo) {
		AutoMove[] stuff = { new PlainMotorMove(robo.armMotors, 0.5, 1.5),
				new Pistons(robo, true), new PlainMotorMove(robo.armMotors, -0.5, 1.5) };
		sequence = new SequentialMove(stuff);
	}
}

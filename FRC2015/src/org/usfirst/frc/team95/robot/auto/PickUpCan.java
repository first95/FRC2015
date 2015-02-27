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
		AutoMove[] stuff = {new TimedArmMove(robo, 1, 0.5), new Pistons(robo, true), 
			new TimedArmMove(robo, -1, 0.5)};
		sequence = new SequentialMove(stuff);
	}
}

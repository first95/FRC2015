package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

/**
 * Takes a can, and then moves right.
 * 
 * @author daroc
 * 
 */
public class TakeCanRight extends PureSequentialMove {

	public TakeCanRight(Robot robot) {
		AutoMove[] moves = { new PickUpCan(robot), new GoRight(robot) };
		sequence = new SequentialMove(moves);
	}

}

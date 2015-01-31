package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

/**
 * Takes a tote, and then moves right.
 * @author daroc
 *
 */

public class TakeToteRight extends PureSequentialMove {
	
	public TakeToteRight(Robot robot) {
		AutoMove[] moves = {new PickUpTote(robot), new GoRight(robot)};
		sequence = new SequentialMove(moves);
	}

}

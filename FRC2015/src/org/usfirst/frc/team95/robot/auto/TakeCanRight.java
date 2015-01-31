package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

/** Takes a can, and then moves right.
 * 
 * @author daroc
 *
 */
public class TakeCanRight extends PureSequentialMove {
	
	public TakeCanRight(Robot robot) {
		AutoMove[] moves = {new PickUpCan(robot), new MoveRelative(robot, 0.75, 0.0, 0.0, 2.0)};
		sequence = new SequentialMove(moves);
	}

}

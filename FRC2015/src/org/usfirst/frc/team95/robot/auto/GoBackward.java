package org.usfirst.frc.team95.robot.auto;

/**
 * Move directly backwards
 */

import org.usfirst.frc.team95.robot.Robot;

public class GoBackward extends PureSequentialMove {
	
	public GoBackward(Robot robot) {
		AutoMove[] vector = {new MoveRelative(robot, 0.0, -0.75, 0.0, 2.0)};
		sequence = new SequentialMove(vector);
	}

}

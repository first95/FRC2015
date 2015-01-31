package org.usfirst.frc.team95.robot.auto;

/**
 * Testing. Performs a squaredance.
 */

import org.usfirst.frc.team95.robot.Robot;

public class Dance extends PureSequentialMove {

	public Dance(Robot robot) {
		AutoMove[] vector = { new MoveRelative(robot, 0.5, 0, 0, 1),
				new MoveRelative(robot, -0.5, 0, 0, 1),
				new MoveRelative(robot, 0, 0.5, 0, 1),
				new MoveRelative(robot, 0, -0.5, 0, 1) };
		sequence = new SequentialMove(vector);
	}

}

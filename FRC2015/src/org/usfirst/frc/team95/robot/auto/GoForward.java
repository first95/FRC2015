package org.usfirst.frc.team95.robot.auto;

/**
 * Move directly forward.
 */

import org.usfirst.frc.team95.robot.Robot;

public class GoForward extends PureSequentialMove {

	public GoForward(Robot robot) {
		AutoMove[] vector = { new MoveRelative(robot, 0.0, 0.75, 0.0, 2.0) };
		sequence = new SequentialMove(vector);
	}

	public GoForward(Robot robot, double speed, double time) {
		AutoMove[] vector = { new MoveRelative(robot, 0.0, speed, 0.0, time) };
		sequence = new SequentialMove(vector);
	}
}
package org.usfirst.frc.team95.robot.auto;

/**
 * Move directly backwards
 */

import org.usfirst.frc.team95.robot.Robot;

public class GoBackward extends PureSequentialMove {

	public GoBackward(Robot robot) {
		AutoMove[] vector = { new MoveRelative(robot, 0.0, -0.5839, 0.0, 3.5) };
		sequence = new SequentialMove(vector);
	}

	public GoBackward(Robot robot, double speed, double time) {
		AutoMove[] vector = { new MoveRelative(robot, 0.0, speed, 0.0, time) };
		sequence = new SequentialMove(vector);
	}

}

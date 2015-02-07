package org.usfirst.frc.team95.robot.auto;

/**
 * Pick up one of the central cans.
 */

import org.usfirst.frc.team95.robot.Robot;

public class GrabLeftCentralCan extends PureSequentialMove {

	public GrabLeftCentralCan(Robot robot) {
		AutoMove[] moves = { new PickUpCan(robot),
				new MoveRelative(robot, 0, -0.75, 0.3, 2),
				new GoBackward(robot), new MakeStack(robot), new NoMove(robot) };
		sequence = new SequentialMove(moves);

	}

}

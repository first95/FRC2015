package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabToteForcefu extends PureSequentialMove {

	public GrabToteForcefu(Robot robot) {
		AutoMove[] moves = { new GoForward(robot, 0.75, 0.5),
				new PickUpTote(robot) };
		sequence = new SequentialMove(moves);
	}

}

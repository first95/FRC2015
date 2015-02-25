package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabMiddleCans extends PureSequentialMove{

	public GrabMiddleCans(Robot robot) {
		AutoMove[] moves = { new GoForward(robot), new PickUpCan(robot), new GoBackward(robot) };
		sequence = new SequentialMove(moves);
	}
}

package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class TakeToteToAutoZone extends PureSequentialMove {

	public TakeToteToAutoZone(Robot robot) {
		AutoMove[] moves = { new PickUpTote(robot), new GoForward(robot) };
		sequence = new SequentialMove(moves);
	}
}

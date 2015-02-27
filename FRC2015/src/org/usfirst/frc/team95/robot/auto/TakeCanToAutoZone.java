package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class TakeCanToAutoZone extends PureSequentialMove {

	public TakeCanToAutoZone(Robot robot) {
		AutoMove[] moves = { new PickUpCan(robot), new GoForward(robot) };
		sequence = new SequentialMove(moves);
	}
}

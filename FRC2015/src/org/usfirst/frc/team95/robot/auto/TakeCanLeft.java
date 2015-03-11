package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class TakeCanLeft extends PureSequentialMove {

	public TakeCanLeft(Robot robot) {
		AutoMove[] moves = { new PickUpCan(robot), new GoLeft(robot) };
		sequence = new SequentialMove(moves);
	}

}

package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabToteForcefully extends PureSequentialMove {
	
	public GrabToteForcefully(Robot robot) {
		AutoMove[] moves = { new GoForward(robot, 0.75, 0.5), new PickUpTote(robot), new GoLeft(robot),
				new GoForward(robot, 0.75, 0.5), new GoRight(robot) } ;
		sequence = new SequentialMove(moves);
	}

}

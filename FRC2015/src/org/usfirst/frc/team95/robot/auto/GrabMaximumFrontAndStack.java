package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

/**
 * Grabs one can, and all three golden totes, before sliding back into the Auto
 * zone.
 * 
 * @author daroc
 * 
 */

public class GrabMaximumFrontAndStack extends PureSequentialMove {

	public GrabMaximumFrontAndStack(Robot robot) {
		AutoMove[] moves = { new PickUpTote(robot),
				new MoveRelative(robot, -0.75, 0, 0, 0.5),
				new PickUpCan(robot),
				new MoveRelative(robot, -0.75, 0, 0, 1.5),
				new PickUpTote(robot), new GoRight(robot),
				new PickUpTote(robot), new GoBackward(robot),
				new MakeStack(robot), new NoMove(robot) };
		sequence = new SequentialMove(moves);
	}

}

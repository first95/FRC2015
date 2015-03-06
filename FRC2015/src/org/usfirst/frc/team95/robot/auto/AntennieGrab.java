package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class AntennieGrab extends PureSequentialMove {

	public AntennieGrab(Robot robot) {
		AutoMove[] moves = { new Pistons(robot.antennie, true), new GoBackward(robot),
				new Pistons(robot.antennie, false) };
		sequence = new SequentialMove(moves);
	}
	
}

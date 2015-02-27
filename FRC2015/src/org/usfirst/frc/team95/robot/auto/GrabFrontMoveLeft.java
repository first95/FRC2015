package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabFrontMoveLeft extends PureSequentialMove {
	
	public GrabFrontMoveLeft(Robot robot) {
		AutoMove[] moves = { new Pistons(robot, true), new PlainMotorMove(robot.armMotors, 0.5, 1.5),
				new MoveRelative(robot, 0.0, 0.75, 0.0, 1.5) };
		
		sequence = new SequentialMove(moves);
		
	}

}

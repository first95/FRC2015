package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class OnTop extends PureSequentialMove {
	
	public OnTop(Robot robot) {
		AutoMove[] moves = { new PlainMotorMove(robot.armMotors, 0.75, 1.5),
				new GoForward(robot, 1.0, 0.25), new PlainMotorMove(robot.armMotors, 0.75, 15.0) };
		sequence = new SequentialMove(moves);
	}

}

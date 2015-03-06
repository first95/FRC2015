package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabFrontMoveBackwards extends PureInterruptedSequentialMove {

	public GrabFrontMoveBackwards(Robot robot) {
		AutoMove[] moves = { new Pistons(robot, true),
				new PlainMotorMove(robot.armMotors, -0.5, 1.5),
				new GoBackward(robot) };

		sequence = new SequentialMove(moves);

		interruptor = robot.armLimitSwitch;

	}
}

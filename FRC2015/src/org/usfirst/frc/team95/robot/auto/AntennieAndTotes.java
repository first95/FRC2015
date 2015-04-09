package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class AntennieAndTotes extends PureSequentialMove {

	public AntennieAndTotes(Robot robot) {
		AutoMove[] moves = { new AutoAlign(robot, 2),
				new Pistons(robot.antennie, true),
				new PlainMotorMove(robot.armMotors, 0.5, 0.75),
				new PlainMotorMove(robot.armMotors, 0.5, 0.75),
				new Pistons(robot.antennie, false),
				new PlainMotorMove(robot.armMotors, 0.5, 2.0),
				new TakeBarrierTotes(robot), new GoBackward(robot) };
		sequence = new SequentialMove(moves);
	}

}

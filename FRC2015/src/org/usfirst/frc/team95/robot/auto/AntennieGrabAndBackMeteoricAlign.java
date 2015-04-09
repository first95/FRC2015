package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class AntennieGrabAndBackMeteoricAlign extends PureSequentialMove {

	public AntennieGrabAndBackMeteoricAlign(Robot robot) {
		AutoMove[] moves = { new GoExactlyForward(robot, 0.5), // Go forward until we slam into the totes, hopefully self-aligning
				new Pistons(robot.antennie, true),
				new PlainMotorMove(robot.armMotors, 0.5, 0.75),
				new PlainMotorMove(robot.armMotors, 0.5, 0.75),
				new GoBackward(robot), new Pistons(robot.antennie, false) };
		sequence = new SequentialMove(moves);
	}

}

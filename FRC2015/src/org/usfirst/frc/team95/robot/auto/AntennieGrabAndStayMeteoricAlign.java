package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class AntennieGrabAndStayMeteoricAlign extends PureSequentialMove {

	public AntennieGrabAndStayMeteoricAlign(Robot robot) {
		AutoMove[] moves = { new GoForward(robot, 0.75, 0.5), // Go forward until we slam into the totes, hopefully self-aligning
				new Pistons(robot.antennie, true),
				new PlainMotorMove(robot.armMotors, 0.5, 0.75),
				new PlainMotorMove(robot.armMotors, 0.5, 0.75)};
		sequence = new SequentialMove(moves);
	}

}

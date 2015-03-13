package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class PlainAntennieGrab extends PureSequentialMove {

	public PlainAntennieGrab(Robot robot) {
		AutoMove[] moves = { new GoForward(robot, 0.75, 0.3), new Pistons(robot.antennie, true),
				new PlainMotorMove(robot.armMotors, 0.5, 0.75), 
				new PlainMotorMove(robot.armMotors, 0.5, 0.75), //new GoBackward(robot),
				new Pistons(robot.antennie, false),
				new PlainMotorMove(robot.armMotors, 0, 0.2) };
		sequence = new SequentialMove(moves);
	}
	
}

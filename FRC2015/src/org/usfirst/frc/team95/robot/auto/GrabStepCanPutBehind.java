package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabStepCanPutBehind extends PureSequentialMove {
	
	public GrabStepCanPutBehind(Robot robot) {
		AutoMove[] moves = { new PlainMotorMove(robot.armMotors, 0.5, 1.5), 
				new GoForward(robot, 0.5, 1.0),
				new Pistons(robot, true), new PlainMotorMove(robot.armMotors, 0, 0.002),
				new PlainMotorMove(robot.armMotors, -0.5, 2.5), 
				new PlainMotorMove(robot.armMotors, 0, 0.2), 
				new Pistons(robot, false), new NoMove(robot)};
		sequence = new SequentialMove(moves);
	}

}

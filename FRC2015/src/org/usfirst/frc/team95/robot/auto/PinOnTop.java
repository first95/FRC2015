package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class PinOnTop extends PureSequentialMove {
	
	public PinOnTop(Robot robot) {
		AutoMove[] simul = { new GoExactlyForward(robot, 2), new PlainMotorMove(robot.armMotors, -0.45, 2) };
		AutoMove[] moves = { new SimultaniousMove(simul), new PlainMotorMove(robot.armMotors, -0.1, 20) };
		sequence = new SequentialMove(moves);
	}

}

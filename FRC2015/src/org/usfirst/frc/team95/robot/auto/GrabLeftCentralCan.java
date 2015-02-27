package org.usfirst.frc.team95.robot.auto;

/**
 * Pick up one of the central cans.
 */

import org.usfirst.frc.team95.robot.Robot;

public class GrabLeftCentralCan extends PureSequentialMove {

	public GrabLeftCentralCan(Robot robot) {
		//AutoMove[] stuff = { new PlainMotorMove(robot.armMotors, -0.5, 0.5), new GoBackward(robot) };
		AutoMove[] moves = { new MoveArmTo(robot, 1.497), new GoForward(robot),
				new Pistons(robot, true), new PlainMotorMove(robot.armMotors, 0, 0.002),
				/*new SimultaniousMove(stuff)*/ new PlainMotorMove(robot.armMotors, -0.5, 2.5), 
				new GoBackward(robot), new PlainMotorMove(robot.armMotors, 0, 0.2), 
				new Pistons(robot, false), new NoMove(robot) };
		sequence = new SequentialMove(moves);

	}

}

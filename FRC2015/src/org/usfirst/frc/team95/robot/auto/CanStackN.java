package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class CanStackN extends PureSequentialMove{
	Robot robot;
	double k;
	public CanStackN(Robot Robo, double h) {
		k = 5*Math.cos(Math.asin(h/5));
		AutoMove[] cans = {new MoveRelative(robot, 0.0, -0.75, 0.0, k), new MoveArmTo(robot, (Math.asin(h/5)))};
		sequence = new SequentialMove(cans);
	}
}

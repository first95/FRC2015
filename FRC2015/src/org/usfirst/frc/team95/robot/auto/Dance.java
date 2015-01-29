package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class Dance extends AutoMove {
	Robot robot;
	SequentialMove sequence;
	
	public Dance(Robot robo) {
		robot = robo;
		AutoMove[] vector = {new MoveRelative(robot, 0.5, 0, 0, 1), new MoveRelative(robot, -0.5, 0, 0, 1),
				new MoveRelative(robot, 0, 0.5, 0, 1), new MoveRelative(robot, 0, -0.5, 0, 1)};
		sequence = new SequentialMove(vector);
	}

	@Override
	public Status init() {
		return sequence.init();
	}

	@Override
	public Status periodic() {
		return sequence.periodic();
	}

	@Override
	public Status stop() {
		return sequence.stop();
	}
	
	

}

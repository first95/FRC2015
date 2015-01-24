package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class TakeToteRight extends AutoMove {
	Robot robot;
	AutoMove sequence;
	
	public TakeToteRight(Robot robo) {
		robot = robo;
		PickUpTote tote;
		tote = new PickUpTote(robot);
		MoveRelative move;
		move = new MoveRelative(robot, -0.75, 0.0, 0.0, 1);
		AutoMove[] vector = {tote, move};
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

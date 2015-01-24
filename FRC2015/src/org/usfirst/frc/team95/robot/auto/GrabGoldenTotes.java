package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabGoldenTotes extends AutoMove {
	Robot robot;
	SequentialMove sequential;
	
	public GrabGoldenTotes(Robot robo) {
		robot = robo;
		AutoMove[] vector = {new PickUpTote(robot), new MoveRelative(robot, 0.75, 0, 0, 1), 
				new PickUpTote(robot), new MoveRelative(robot, 0.75, 0, 0, 1),
				new PickUpTote(robot), new MoveRelative(robot, 0.75, 0, 0, 1),
				new GoBackward(robot), new NoMove(robot)};
		sequential = new SequentialMove(vector);
	}

	@Override
	public Status init() {
		return sequential.init();
	}

	@Override
	public Status periodic() {
		return sequential.periodic();
	}

	@Override
	public Status stop() {
		return sequential.stop();
	}

}

package org.usfirst.frc.team95.robot.auto;

/**
 * Grab all the golden totes.
 */

import org.usfirst.frc.team95.robot.Robot;

public class GrabGoldenTotes extends AutoMove {
	Robot robot;
	SequentialMove sequential;
	boolean stopped;

	public GrabGoldenTotes(Robot robo) {
		robot = robo;
		AutoMove[] vector = { new TakeToteRight(robot),
				new TakeToteRight(robot), new TakeToteRight(robot),
				new GoBackward(robot), new MakeStack(robot), new NoMove(robot) };
		sequential = new SequentialMove(vector);
		stopped = false;
	}

	@Override
	public Status init() {
		return sequential.init();
	}

	@Override
	public Status periodic() {
		System.out.println("Grab Golden Totes!");
		Status status = Status.emergency;
		if (!stopped) {
			status = sequential.periodic();
		}
		if (status == Status.isNotAbleToContinue
				|| status == Status.isAbleToContinue) {
			stopped = true;
		}
		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		return sequential.stop();
	}

}

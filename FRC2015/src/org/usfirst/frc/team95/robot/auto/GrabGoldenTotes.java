package org.usfirst.frc.team95.robot.auto;

import java.util.Vector;

import org.usfirst.frc.team95.robot.Robot;

public class GrabGoldenTotes extends AutoMove {
	Robot robot;
	Vector<AutoMove> vector;
	SequentialMove sequential;
	
	public GrabGoldenTotes(Robot robo) {
		robot = robo;
		vector = new Vector<AutoMove>();
		vector.add(new TakeToteRight(robot));
		vector.add(new TakeToteRight(robot));
		vector.add(new TakeToteRight(robot));
		vector.add(new GoBackward(robot));
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

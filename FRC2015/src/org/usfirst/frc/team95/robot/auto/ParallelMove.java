package org.usfirst.frc.team95.robot.auto;

public class ParallelMove extends AutoMove {
	AutoMove[] moves;
	
	public ParallelMove(AutoMove[] moves) {
		this.moves = moves;
	}

	@Override
	public Status init() {
		for (int i = 0; i < moves.length; i++) {
			moves[i].init();
		}
		return Status.needsToContinue;
	}

	@Override
	public Status periodic() {
		for (int i = 0; i < moves.length; i++) {
			moves[i].periodic();
		}
		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		for (int i = 0; i < moves.length; i++) {
			moves[i].stop();
		}
		return Status.isNotAbleToContinue;
	}

}

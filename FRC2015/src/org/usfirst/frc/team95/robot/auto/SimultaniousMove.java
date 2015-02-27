package org.usfirst.frc.team95.robot.auto;

public class SimultaniousMove extends AutoMove {
	AutoMove[] moves;
	boolean finished1, finished2;
	
	public SimultaniousMove(AutoMove[] moves) {
		this.moves = moves;
	}
	
	public Status init() {
		moves[0].init();
		moves[1].init();
		
		return Status.wantsToContinue;
	}
	
	public Status periodic() {
		
		Status st1 = moves[0].periodic();
		if (st1 == Status.isNotAbleToContinue || st1 == Status.isAbleToContinue) {
			finished1 = true;
		}
		Status st2 = moves[0].periodic();
		if (st2 == Status.isNotAbleToContinue || st2 == Status.isAbleToContinue) {
			finished2 = true;
		}
		
		if (finished1 && finished2) {
			return Status.isAbleToContinue;
		}
		return Status.wantsToContinue;
	}
	
	public Status stop() {
		moves[0].stop();
		moves[1].stop();
		return Status.isNotAbleToContinue;
	}

}

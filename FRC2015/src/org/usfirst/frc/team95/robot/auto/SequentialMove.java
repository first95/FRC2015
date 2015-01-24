package org.usfirst.frc.team95.robot.auto;

import java.util.Vector;

public class SequentialMove extends AutoMove {
	Vector<AutoMove> moves;
	int index;
	
	public SequentialMove(Vector<AutoMove> movesToPerform) {
		moves = movesToPerform;
		index = 0;
	}
	
	public Status init() {
		return moves.get(index).init();
	}
	
	public Status periodic() {
		Status status = moves.get(index).periodic();
		if (status == Status.isAbleToContinue || status == Status.isNotAbleToContinue) {
			moves.get(index).stop();
			index += 1;
			if (index >= moves.size()) {
				this.stop();
				return Status.isNotAbleToContinue;
			}
		} else if (status == Status.emergency) {
			moves.get(index).stop();
			return Status.emergency;
		}
	}
	
	public Status stop() {
		;
	}

}

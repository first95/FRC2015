package org.usfirst.frc.team95.robot.auto;

public class SequentialMove extends AutoMove {
	AutoMove[] moves;
	int index;
	
	public SequentialMove(AutoMove[] movesToPerform) {
		moves = movesToPerform;
		index = 0;
		
	}
	
	public Status init() {
		index = 0;
		return moves[index].init();
	}
	
	public Status periodic() {
		try {
			Status status = moves[index].periodic();
			if (status == Status.isAbleToContinue || status == Status.isNotAbleToContinue) {
				moves[index].stop();
				index += 1;
				if (index < moves.length - 1) {
					return moves[index].init();
				} else {
					return stop();
				}
			} else if (status == Status.emergency) {
				moves[index].stop();
				return Status.emergency;
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			System.out.println("Got out of array.");
			return Status.isNotAbleToContinue;
		}
		return Status.needsToContinue;
	}
	
	public Status stop() {
		return Status.isNotAbleToContinue;
	}

}

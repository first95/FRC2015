package org.usfirst.frc.team95.robot.auto;

/**
 * Performs automoves sequentially.
 * @author daroc
 *
 */

public class SequentialMove extends AutoMove {
	AutoMove[] moves;
	int william;
	
	public SequentialMove(AutoMove[] movesToPerform) {
		
		moves = movesToPerform;
		william = 0;
		
	}
	
	public Status init() {
		System.out.println("Sequential Move init");
		william = 0;
		return moves[william].init();
	}
	
	public Status periodic() {
		System.out.println("My array is "+moves.length+" long. "+ william);
		System.out.println("Better is more."+moves.length+" long. "+ william);
			Status status = moves[william].periodic();
		System.out.println("More is better."+moves.length+" long. "+ william);
		//for (int i = 0; i < william; i++) {
		//	System.out.print("+");
		//}
		System.out.println("");
		
			if (status == Status.isAbleToContinue || status == Status.isNotAbleToContinue) {
				System.out.println("Stopword");
				moves[william].stop();
				william += 1;
				if (william < (moves.length)) {
					System.out.println("Sufficiently small index");
					return moves[william].init();
				} else {
					System.out.println("Insufficiently small index");
					return stop();
				}
			} else if (status == Status.emergency) {
				System.out.println("Emergency! Emergency! Everybody to get from street.");
				moves[william].stop();
				return Status.emergency;
			}
		return Status.needsToContinue;
	}
	
	public Status stop() {
		return Status.isNotAbleToContinue;
	}

}

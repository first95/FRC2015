package org.usfirst.frc.team95.robot.auto;

/**
 * Parent of auto-moves that are merely packaged sequential moves.
 * @author daroc
 *
 */

public class PureSequentialMove extends AutoMove {

	SequentialMove sequence = null;
	
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

package org.usfirst.frc.team95.robot.auto;

/**
 * Parent of all autonomous moves. Requires certain basic functionalities.
 * 
 * @author daroc
 * 
 */

public abstract class AutoMove {

	public abstract Status init();

	public abstract Status periodic();

	public abstract Status stop();

	public enum Status {
		needsToContinue, wantsToContinue, isAbleToContinue, isNotAbleToContinue, emergency
	}

}

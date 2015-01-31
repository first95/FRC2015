package org.usfirst.frc.team95.robot.auto;

public abstract class AutoMove {
	
	public abstract Status init();
	
	public abstract Status periodic();
	
	public abstract Status stop();
	
	enum Status {
		needsToContinue,
		wantsToContinue,
		isAbleToContinue,
		isNotAbleToContinue,
		emergency
	}

}

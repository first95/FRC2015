package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;

public class TakeBarrierTotes extends PureSequentialMove {

	public TakeBarrierTotes(Robot robot) {
		AutoMove[] moves = {
				new MoveFauxPIDTo(robot.fingerController,
						RobotConstants.kFingerSetpoints[0]),
				new GoForward(robot, 0.75, 0.5),
				new MoveFauxPIDTo(robot.fingerController,
						RobotConstants.kFingerSetpoints[2]),
				new GoBackward(robot) };
		sequence = new SequentialMove(moves);
	}

}

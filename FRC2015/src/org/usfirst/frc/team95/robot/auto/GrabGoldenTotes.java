package org.usfirst.frc.team95.robot.auto;

/**
 * Grab all the golden totes.
 */

import org.usfirst.frc.team95.robot.Robot;

public class GrabGoldenTotes extends PureSequentialMove {
	Robot robot;

	public GrabGoldenTotes(Robot robo) {
		robot = robo;
		AutoMove[] vector = { new GrabToteForcefully(robo),
				new GrabToteForcefully(robo), new GrabToteForcefully(robo),
				new GoRight(robo), new MakeToteStack(robo), 
				new GoBackward(robo, 0.5, 0.25), new NoMove(robot) } ;
		/*AutoMove[] vector = { new TakeToteRight(robot),
				new TakeToteRight(robot), new TakeToteRight(robot),
				new GoBackward(robot), new MakeStack(robot), new NoMove(robot) }; */
		sequence = new SequentialMove(vector);
	}

}

// GrabToteForcefullly x 3, GoRight, PutDownTotes, GoBack (a little), NoMove
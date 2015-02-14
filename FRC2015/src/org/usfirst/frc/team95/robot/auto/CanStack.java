package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;
import edu.wpi.first.wpilibj.Timer;

public class CanStack extends CanStackN {
	Robot robot;
	int stage = 0;

	public CanStack(Robot robo) {
		super(robo,1);
	}
}

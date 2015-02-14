package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;
import org.usfirst.frc.team95.robot.RobotConstants;
import edu.wpi.first.wpilibj.Timer;

public class CanStack2 extends CanStackN {
	Robot robot;
	int stage = 0;

	public CanStack2(Robot robo) {
		super(robo,2);
	}
}

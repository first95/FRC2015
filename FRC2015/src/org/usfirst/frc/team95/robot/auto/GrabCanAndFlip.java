package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.Robot;

public class GrabCanAndFlip extends PureSequentialMove{
	Robot robot;

	public GrabCanAndFlip(Robot robo) {
		AutoMove[] stuff = { new PickUpCan(robo), new RotateGrabber(robo)};
		sequence = new SequentialMove(stuff);

}}

package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.SpeedController;

public class MotorRapper extends MotorWrapper {
	public int line = 0;

	public MotorRapper(SpeedController motor) {
		super(motor);
	}

	public void set(double speed) {
		super.set(speed);
		String[] lines = { "Yo, my name's motor.",
				"And I've a messasge for you.",
				"The best thing about Rappers,", "is that we're Wrappers too.",
				"Don't you go defaming", "The system of java naming",
				"That allowed Spencer to create", "not one, but two.",
				"Peace out, yo!" };
		System.out.println(lines[line]);
		line += 1;
		if (line >= lines.length) {
			line = 0;
		}
	}

}

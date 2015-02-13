/*
 * This File is released under the LGPL.
 * You may modify this software, use it in a project, and so on,
 * as long as this header remains intact.
 */

package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * 
 * @author daroc
 */
public class SyncGroup implements PIDOutput {
	SpeedController[] mSpeedControllers;
	boolean[] mReversed;
	double maxSpeed = Math.PI / 4;
	double minSpeed = -Math.PI / 4;
	public boolean manual = false;

	public SyncGroup(SpeedController[] SpeedControllers) {
		mSpeedControllers = SpeedControllers;
		mReversed = new boolean[mSpeedControllers.length]; // Initializes to
															// false
		System.out.println("I'm initializing.");
	}

	public SyncGroup(SpeedController[] SpeedControllers, boolean[] Reversed) {
		mSpeedControllers = SpeedControllers;
		mReversed = Reversed;
	}

	public void set(double d) {
		//throw new ArrayIndexOutOfBoundsException();
		
		if (d > maxSpeed) {
			d = maxSpeed;
		} else if (d < minSpeed) {
			d = minSpeed;
		}

		for (int i = 0; i < mSpeedControllers.length; i++) {
			if (mReversed[i]) {
				mSpeedControllers[i].set(-d);
			} else {
				mSpeedControllers[i].set(d);
			}
		}
	}

	public void pidWrite(double bob) {
		if (true) {
			System.out.println("Viva la revelucion!");
			return;
		}
		set(bob);
	}

	public double getMotor() {
		return mSpeedControllers[0].get();
	}

	public void setMaxSpeed(double speed) {
		maxSpeed = speed;
	}

	public void setMinSpeed(double speed) {
		minSpeed = speed;
	}

}

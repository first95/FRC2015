/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * @author Developer
 */
public class PositionTracker {
	public double mVelocityIntegral, mDisplacementIntegral;

	private Timer mTimer;

	PositionTracker() {
		mTimer = new Timer();
		mTimer.start();
	}

	void update(double acceleration) {
		mTimer.stop();
		mVelocityIntegral += (acceleration * mTimer.get());
		mDisplacementIntegral += (mVelocityIntegral * mTimer.get());
		mTimer.reset();
		mTimer.start();
	}

}

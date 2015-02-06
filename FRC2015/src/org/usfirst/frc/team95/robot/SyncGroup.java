/*
 * This File is released under the LGPL.
 * You may modify this software, use it in a project, and so on,
 * as long as this header remains intact.
 */

package org.usfirst.frc.team95.robot;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 *
 * @author daroc
 */
public class SyncGroup implements PIDOutput {
    SpeedController[] mSpeedControllers;
    Solenoid[] mSolenoids;
    boolean[] mReversed;
    
    public SyncGroup (SpeedController[] SpeedControllers) {
        mSpeedControllers = SpeedControllers;
        mReversed = new boolean[mSpeedControllers.length]; //Initializes to false
        System.out.println("I'm initializing.");
    }
    
    public SyncGroup (Solenoid[] Solenoids) {
        mSolenoids = Solenoids;
        mReversed = new boolean[mSolenoids.length]; //Initializes to false
    }
    
    public SyncGroup (SpeedController[] SpeedControllers, boolean[] Reversed) {
        mSpeedControllers = SpeedControllers;
        mReversed = Reversed;
    }
    
    public SyncGroup (Solenoid[] Solenoids, boolean[] Reversed) {
        mSolenoids = Solenoids;
        mReversed = Reversed;
    }
    
    
    public void set(double d) {
    	System.out.println("Set");
        for (int i = 0; i < mSpeedControllers.length;  i++) {
            if (mReversed[i]) {
                mSpeedControllers[i].set(-d);
            } else {
                mSpeedControllers[i].set(d);
            }
        }
    }
    
    public void pidWrite(double bob) {
    	set(bob);
    }
    
    public void set(boolean b) {
        for (int i = 0; i < mSolenoids.length; i++) {
            if (mReversed[i]) {
                mSolenoids[i].set(!b);
            } else {
                mSolenoids[i].set(b);
            }
        }
    }
    
    public boolean getPiston() {
        return mSolenoids[0].get();
    }
    
    public double getMotor() {
        return mSpeedControllers[0].get();
    }
    
}

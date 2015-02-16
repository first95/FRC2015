package org.usfirst.frc.team95.robot.auto;

import org.usfirst.frc.team95.robot.RobotConstants;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class FauxPID extends AutoMove {
	public double mP, mI, mD, mSetpoint;
	public boolean enabled;
	double mIntegral, mPrevError;
	PIDSource mSource;
	PIDOutput mOutput;
	
	public FauxPID(double p, double i, double d, PIDSource source, PIDOutput output) {
		mP = p;
		mI = i;
		mD = d;
		mSource = source;
		mOutput = output;
		enabled = true;
	}
	

	@Override
	public Status init() {
		mIntegral = 0;
		mPrevError = 0;
		return Status.needsToContinue;
	}

	@Override
	public Status periodic() {
		if (!enabled) {
			return Status.wantsToContinue;
		}
		
		double input = mSource.pidGet();
		double error = mSetpoint - input;
		if (mI != 0) {
            double potentialIGain = (mIntegral + error) * mI;
            if (potentialIGain < 1.0) {
                if (potentialIGain > -1.0) {
                    mIntegral += error;
                } else {
                    mIntegral = -1.0 / mI;
                }
            } else {
                mIntegral = 1.0 / mI;
            }
        }
		
		mOutput.pidWrite(error * mP + mIntegral * mI + (error - mPrevError) * mD);
		mPrevError = error;
		
		return Status.wantsToContinue;
	}

	@Override
	public Status stop() {
		return Status.isNotAbleToContinue;
	}
	
	public void setSetpoint(double setpoint) {
		mSetpoint = setpoint;
	}
	
	public boolean onTarget() {
		return Math.abs(mSetpoint - mSource.pidGet()) < RobotConstants.kPIDTolerance;
	}

}

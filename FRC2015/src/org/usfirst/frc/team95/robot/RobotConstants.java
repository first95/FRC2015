package org.usfirst.frc.team95.robot;

public class RobotConstants {
	public static final double kMotorSpeedChangeMaximum = 0.5;
	public static final int kFrontLeftMotor = 9;
	public static final int kFrontRightMotor = 0;
	public static final int kBackLeftMotor = 8;
	public static final int kBackRightMotor = 1;
	public static final int kLeftArmMotor = 7;
	public static final int kRightArmMotor = 2;
	public static final int kFingerMotor = 6;
	public static final int kArmEncoder = 8;
	public static final int kFingerEncoder = 6;
	public static final int kGyro = 0;
	public static final int kChasis = 0;
	public static final int kWeapons = 1;
	public static final int kChangeDriveStyle = 11;
	public static final int kArmOverride = 2;
	public static final int kCalibrationLength = 500;
	public static final int kAntennieButton = 3;
	public static final double kArmP = 0.042;
	public static final double kArmI = 0;
	public static final double kArmD = 0;
	public static final double kFingerP = 0.17;
	public static final double kFingerI = 0.012;
	public static final double kFingerD = 0.0;
	public static final double kPIDUpdateInterval = 50.0;
	public static final double[] kFingerSetpoints = { -17.973, 8, 10, 23, 25,
			38, 40 };
	// Aquire: -17.973
	//
	public static final double kDeadband = 0.1;
	public static final double kTurningCloseness = 1;
	public static final double kTurningTimeoute = 3;
	public static final double kArmPositionGrab = -0.1;
	public static final double kArmPositionZenith = Math.PI / 2;
	public static final double kArmPositionBehind = Math.PI;
	public static final int kArmPistons = 0;
	public static final int kAntennie = 2;
	public static final int kArmPistonsButton = 1;
	public static final int kGrabberRotatePiston = 4;
	public static final int kGrabberRotateButton = 4;
	public static final double kArmTolerance = 0.1;
	public static final double kFingerTolerance = 0.2;
	public static final int kPCMId = 1;
	public static final int kPDPId = 0;
	public static final double kArmEncoderPulseDistance = -Math.PI / 256; // Radians
	public static final double kFingerEncoderPulseDistance = Math.PI / 128; // Inches
	public static final double kArmLimitedSpeed = 0.7;
	public static final double kArmDistanceP = 0;
	public static final double kArmDistanceI = 0;
	public static final double kArmDistanceD = 0;
	public static final double kTippynessTolerance = 20;
	public static final int kArmLimitSwitch = 0;
	public static final double kArmLimitSwitchSloppyness = 0.03;
	public static final double kRotationTolerance = 5;
	public static final double kMaxRotationSpeed = 1800 / Math.PI; // degrees /
																	// sec
	public static final int kLeftArmMotorCurrent = 15;
	public static final int kRightArmMotorCurrent = 0;
	public static final double kArmMotorDifferenceTolerance = 0.1;
	public static final int kResetRateButton = 1;
	public static final double kPIDTolerance = 2;
	public static final int kTopFingerLimitSwitch = 1;
	public static final int kLowFingerLimitSwitch = 2;
	public static final int kMidLowFingerLimitSwitch = 5;
	public static final int kMidHighFingerLimitSwitch = 4;

	public static final double kStraightAlignmentDeadband = 1;
	public static final double kSensorCloseness = 4;
	public static final double kSideDistanceLength = 4;

	public static double sensorVoltageToCm(double voltage) {
		return 11.75 / (voltage - 0.0625) - 0.42;
	}

}

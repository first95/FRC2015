package org.usfirst.frc.team95.robot;

public class RobotConstants {
	public static double kMotorSpeedChangeMaximum = 0.5;
	public static int kFrontLeftMotor = 9;
	public static int kFrontRightMotor = 0;
	public static int kBackLeftMotor = 8;
	public static int kBackRightMotor = 1;
	public static int kLeftArmMotor = 7;
	public static int kRightArmMotor = 2;
	public static int kFingerMotor = 6;
	public static int kFrontLeftEncoder = 0;
	public static int kFrontRightEncoder = 2;
	public static int kBackLeftEncoder = 4;
	public static int kBackRightEncoder = 6;
	public static int kArmEncoder = 8;
	public static int kFingerEncoder = 23;
	public static int kGyro = 0;
	public static int kChasis = 0;
	public static int kWeapons = 1;
	public static int kChangeDriveStyle = 2;
	public static int kFieldCentric = 5;
	public static int kRotate90Left = 3;
	public static int kRotate90Right = 4;
	public static int kAutoStack = 7;
	public static int kAutoStackCan = 8;
	public static int kAutoGrabCan = 9;
	public static int kCalibrationLength = 500;
	public static double kArmP = 1;
	public static double kArmI = 0.1;
	public static double kArmD = 0.01;
	public static double kFingerP = 0.0;
	public static double kFingerI = 0.0;
	public static double kFingerD = 0.0;
	public static double kPIDUpdateInterval = 50.0;
	public static double[] kFingerSetpoints = { 0, 0, 0, 0, 0, 0 };
	public static double kDeadband = 0.04;
	public static double kTurningCloseness = 1;
	public static double kTurningTimeoute = 5;
	public static double kArmPositionGrab = 0;
	public static double kArmPositionZenith = Math.PI / 4;
	public static double kArmPositionBehind = Math.PI / 2;
	public static int kArmPistons = 0;
	public static int kArmPistonsButton = 7;
	public static double kArmTolerance = 0.1;
	public static double kFingerTolerance = 0.2;
	public static int kPCMId = 1;
	public static int kPDPId = 0;
	public static double kArmEncoderPulseDistance = Math.PI / 256;
	public static double kFingerEncoderPulseDistance = 1;
	public static double kArmLimitedSpeed = 0.7;
	public static double kArmDistanceP = 0;
	public static double kArmDistanceI = 0;
	public static double kArmDistanceD = 0;
	public static double kTippynessTolerance = 20;
}

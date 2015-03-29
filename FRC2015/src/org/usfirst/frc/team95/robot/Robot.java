/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team95.robot;

import org.usfirst.frc.team95.robot.auto.AntennieAndTotes;
import org.usfirst.frc.team95.robot.auto.AntennieGrabAndBack;
import org.usfirst.frc.team95.robot.auto.AntennieGrabAndBackMeteoricAlign;
import org.usfirst.frc.team95.robot.auto.AntennieGrabAndStayMeteoricAlign;
import org.usfirst.frc.team95.robot.auto.GrabCanAndFlip;
import org.usfirst.frc.team95.robot.auto.GrabStepCan;
import org.usfirst.frc.team95.robot.auto.PlainAntennieGrab;
import org.usfirst.frc.team95.robot.auto.AutoMove;
import org.usfirst.frc.team95.robot.auto.AutoMove.Status;
import org.usfirst.frc.team95.robot.auto.CanStack;
import org.usfirst.frc.team95.robot.auto.CanStack2;
import org.usfirst.frc.team95.robot.auto.CanStack3;
import org.usfirst.frc.team95.robot.auto.Dance;
import org.usfirst.frc.team95.robot.auto.FauxPID;
import org.usfirst.frc.team95.robot.auto.GoBackward;
import org.usfirst.frc.team95.robot.auto.GrabCanFromFloor;
import org.usfirst.frc.team95.robot.auto.GrabFrontMoveBackwards;
import org.usfirst.frc.team95.robot.auto.GrabFrontMoveForwards;
import org.usfirst.frc.team95.robot.auto.GrabFrontMoveLeft;
import org.usfirst.frc.team95.robot.auto.GrabFrontMoveRight;
import org.usfirst.frc.team95.robot.auto.GrabGoldenTotes;
import org.usfirst.frc.team95.robot.auto.GrabLeftCentralCan;
import org.usfirst.frc.team95.robot.auto.GrabMaximumFrontAndStack;
import org.usfirst.frc.team95.robot.auto.GrabStepCanPutBehind;
import org.usfirst.frc.team95.robot.auto.GrabStepCanPutFront;
import org.usfirst.frc.team95.robot.auto.MakeStack;
import org.usfirst.frc.team95.robot.auto.NoMove;
import org.usfirst.frc.team95.robot.auto.PickUpCan;
import org.usfirst.frc.team95.robot.auto.PickUpTote;
import org.usfirst.frc.team95.robot.auto.PlainMotorMove;
import org.usfirst.frc.team95.robot.auto.QuickBarrierGrab;
import org.usfirst.frc.team95.robot.auto.TakeToteRight;
import org.usfirst.frc.team95.robot.auto.GoForward;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.smartdashboard.

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
@SuppressWarnings("unused")
// So that it doesn't complain about unused imports
public class Robot extends IterativeRobot {
	/*
	 * This is the definitions section, where all variables that need to persist
	 * are declared to exist.
	 */

	Victor frontLeft, frontRight, backLeft, backRight, fingerTalon;
	Talon leftArmTalon, rightArmTalon;
	public MotorWrapper realFrontLeft, realFrontRight, realBackLeft,
			realBackRight, realLeftArmMotor, realRightArmMotor;
	public ResetableEncoder armEncoder, fingerEncoder;
	public RobotDrive driveTrain;

	public SyncGroup armMotors;

	public ResetableGyro gyro;
	ADXL345_I2C extraAccel;
	BuiltInAccelerometer builtinAccel;
	public CommonFilter accel;

	PositionTracker xDisplacement, yDisplacement, zDisplacement;

	Joystick chasis, weapons;

	ButtonTracker changeDriveStyle, overrideTracker, blue1, blue2, blue3,
			blue4, blue5, blue6, triggerButton, stopSpin, upTurnSpeed,
			antennieButton, grabberRotateButton, toteUp, toteDown;

	boolean driveStyle, rotating, fingersAtBottom, fieldcentric = false;
	boolean armForwards = false;
	boolean armBackwards = false;
	double targetAngle;

	Timer timeOut;
	Timer timeLag;

	PowerDistributionPanel powerDistribution;

	double[] xAccelCalibration, yAccelCalibration, zAccelCalibration,
			xGyroCalibration, yGyroCalibration, zGyroCalibration;

	double xAccelMean, yAccelMean, zAccelMean, xGyroMean, GyroMean, zGyroMean;

	public FauxPID armController, fingerController;

	public DoubleSolenoid antennie, grabberRotatePiston;
	public Solenoid armPistons;

	AutoMove autoMove;

	SendableChooser chooser;
	private boolean autoStopped, fingerDangerousTerritory, hasHitBouncy;

	Compressor compressor;

	TippynessMeasure tipsyness, swayfulness;

	public DigitalInput armLimitSwitch, topFingerLimitSwitch,
			lowFingerLimitSwitch, midLowFingerLimitSwitch,
			midHighFingerLimitSwitch;
	public MotorWrapper realFingerMotor;

	public Timer bouncyTimeOut, downfulnessTimeOut, upfulnessTimeOut,
			startedMovingTimeOut;

	ButtonTracker smallUp, smallDown, largeUp, largeDown;

	boolean grippersLatched = false;

	AutoMove lastSeenAutoMove = null;

	// false is slow turn
	boolean turnSpeed = false;

	enum MovingState {
		up, still, down
	}

	MovingState movingIndependantly;

	AnalogInput forwardLeft, forwardRight, forwardMiddle, forwardOffCenter;
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		/*
		 * This sets up everything's values.
		 */
		frontLeft = new Victor(RobotConstants.kFrontLeftMotor);
		frontRight = new Victor(RobotConstants.kFrontRightMotor);
		backLeft = new Victor(RobotConstants.kBackLeftMotor);
		backRight = new Victor(RobotConstants.kBackRightMotor);
		leftArmTalon = new Talon(RobotConstants.kLeftArmMotor);
		rightArmTalon = new Talon(RobotConstants.kRightArmMotor);
		fingerTalon = new Victor(RobotConstants.kFingerMotor);
		realFrontLeft = new MotorWrapper(frontLeft);
		// realFrontLeft.scaling = 0.9;
		realFrontRight = new MotorWrapper(frontRight);
		// realFrontRight.scaling = 0.9;
		realBackLeft = new MotorWrapper(backLeft);
		realBackRight = new MotorWrapper(backRight);
		// realBackLeft.scaling = 0.9;
		realRightArmMotor = new MotorWrapper(rightArmTalon);

		realLeftArmMotor = new MotorWrapper(leftArmTalon);

		realFingerMotor = new MotorWrapper(fingerTalon);

		armEncoder = new ResetableEncoder(RobotConstants.kArmEncoder,
				RobotConstants.kArmEncoder + 1);
		armEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
		armEncoder.setDistancePerPulse(RobotConstants.kArmEncoderPulseDistance);
		fingerEncoder = new ResetableEncoder(RobotConstants.kFingerEncoder,
				RobotConstants.kFingerEncoder + 1);
		fingerEncoder
				.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
		fingerEncoder
				.setDistancePerPulse(RobotConstants.kFingerEncoderPulseDistance);
		driveTrain = new RobotDrive(realFrontLeft, realBackLeft,
				realFrontRight, realBackRight);
		driveTrain.setInvertedMotor(MotorType.kFrontLeft, true);
		driveTrain.setInvertedMotor(MotorType.kRearLeft, true);

		//gyro = new ResetableGyro(RobotConstants.kGyro);
		extraAccel = new ADXL345_I2C(Port.kOnboard, Accelerometer.Range.k8G);
		builtinAccel = new BuiltInAccelerometer();
		accel = new CommonFilter(extraAccel, builtinAccel);

		xDisplacement = new PositionTracker();
		yDisplacement = new PositionTracker();
		zDisplacement = new PositionTracker();

		chasis = new Joystick(RobotConstants.kChasis);
		weapons = new Joystick(RobotConstants.kWeapons);

		changeDriveStyle = new ButtonTracker(chasis,
				RobotConstants.kChangeDriveStyle);

		overrideTracker = new ButtonTracker(chasis, RobotConstants.kArmOverride);
		antennieButton = new ButtonTracker(weapons,
				RobotConstants.kAntennieButton);
		grabberRotateButton = new ButtonTracker(weapons,
				RobotConstants.kGrabberRotateButton);
		driveStyle = false; // False == traditional
		blue1 = new ButtonTracker(chasis, 5);
		blue2 = new ButtonTracker(chasis, 6);
		blue3 = new ButtonTracker(chasis, 7);
		blue4 = new ButtonTracker(chasis, 8);
		blue5 = new ButtonTracker(chasis, 9);
		blue6 = new ButtonTracker(chasis, 10);
		upTurnSpeed = new ButtonTracker(chasis, 2);
		triggerButton = new ButtonTracker(weapons,
				RobotConstants.kArmPistonsButton);
		stopSpin = new ButtonTracker(chasis, RobotConstants.kResetRateButton);
		toteUp = new ButtonTracker(chasis, 11);
		toteDown = new ButtonTracker(chasis, 16);

		rotating = false;
		fieldcentric = true;

		timeOut = new Timer();
		timeLag = new Timer();
		powerDistribution = new PowerDistributionPanel();

		xAccelCalibration = new double[RobotConstants.kCalibrationLength];
		yAccelCalibration = new double[RobotConstants.kCalibrationLength];
		zAccelCalibration = new double[RobotConstants.kCalibrationLength];

		SpeedController[] table = { realLeftArmMotor, realRightArmMotor };
		boolean[] reversed = { false, true };
		armMotors = new SyncGroup(table, reversed);
		table = null;

		armController = new FauxPID(RobotConstants.kArmP, RobotConstants.kArmI,
				RobotConstants.kArmD, armEncoder, armMotors);
		// armController.enable();
		// armController.setTolerance(1.0);
		armEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);

		fingerController = new FauxPID(RobotConstants.kFingerP,
				RobotConstants.kFingerI, RobotConstants.kFingerD,
				fingerEncoder, realFingerMotor);
		// fingerController.setAbsoluteTolerance(RobotConstants.kFingerTolerance);
		// fingerController.enable();

		armPistons = new Solenoid(RobotConstants.kPCMId, 2);

		tipsyness = new TippynessMeasure();
		swayfulness = new TippynessMeasure();

		armLimitSwitch = new DigitalInput(RobotConstants.kArmLimitSwitch);
		topFingerLimitSwitch = new DigitalInput(
				RobotConstants.kTopFingerLimitSwitch);
		lowFingerLimitSwitch = new DigitalInput(
				RobotConstants.kLowFingerLimitSwitch);
		midLowFingerLimitSwitch = new DigitalInput(
				RobotConstants.kMidLowFingerLimitSwitch);
		midHighFingerLimitSwitch = new DigitalInput(
				RobotConstants.kMidHighFingerLimitSwitch);

		bouncyTimeOut = new Timer();
		downfulnessTimeOut = new Timer();
		upfulnessTimeOut = new Timer();
		startedMovingTimeOut = new Timer();

		smallUp = new ButtonTracker(chasis, 5);
		smallDown = new ButtonTracker(chasis, 10);
		largeUp = new ButtonTracker(chasis, 6);
		largeDown = new ButtonTracker(chasis, 9);

		antennie = new DoubleSolenoid(RobotConstants.kPCMId, 0, 7);
		grabberRotatePiston = new DoubleSolenoid(RobotConstants.kPCMId, 1, 6);

		compressor = new Compressor();
		compressor.start();

		forwardLeft = new AnalogInput(3);
		forwardRight = new AnalogInput(0);
		forwardMiddle = new AnalogInput(2);
		forwardOffCenter = new AnalogInput(1);
		
		try {
//			CameraServer.getInstance().startAutomaticCapture("cam0");
		} catch (Exception e){
			System.out.println("Failed to initialize camera!");
		}
		chooser = new SendableChooser();
		chooser.addObject("Zombie", new NoMove(this));
		// chooser.addObject("TakeToteRight", new TakeToteRight(this));
//		chooser.addObject("TakeGoldenTotes", new GrabGoldenTotes(this));
		chooser.addObject("Plain Antennie", new PlainAntennieGrab(this));
		// chooser.addObject("Antennie And Totes", new AntennieAndTotes(this));
		chooser.addObject("Antennie autoalign grab and Move Back", new AntennieGrabAndBack(this));
		chooser.addObject("Antennie slam and stay", new AntennieGrabAndStayMeteoricAlign(this));
		chooser.addObject("Antennie slam and back", new AntennieGrabAndBackMeteoricAlign(this));
		// chooser.addObject("Dance", new Dance(this));
		// chooser.addObject("GrabMaximumFrontAndStack",
		// new GrabMaximumFrontAndStack(this));
		chooser.addObject("Grab Barrier Can and Move Back",
				new GrabLeftCentralCan(this));
		chooser.addObject("Grab Barrier Can",
				new GrabStepCan(this));
		// chooser.addObject("Move the Arm", new PlainMotorMove(armMotors, 0.25,
		// 1.0));
		// chooser.addObject("Floor Can", new GrabCanFromFloor(this));
		chooser.addObject("Move Forward", new GoForward(this));
//		chooser.addObject("Already Grabbed Can and Move Left",
//				new GrabFrontMoveLeft(this));
//		chooser.addObject("Already Grabbed Can and Move Right",
//				new GrabFrontMoveRight(this));
		// chooser.addObject("Grab Can and Move Forward", new
		// GrabFrontMoveForwards(this));
//		chooser.addObject("Already Grabbed Can and Move Backward",
//				new GrabFrontMoveBackwards(this));
		// chooser.addObject("Grab Barrier Can and Place Behind",
		// new GrabStepCanPutBehind(this));
		chooser.addObject("Grab Barrier Can and Place Front",
				new GrabStepCanPutFront(this));
		chooser.addObject("Grab Barrier Can and Flip", new GrabCanAndFlip(this));
		chooser.addObject("Quick Barrier Grab", new QuickBarrierGrab(this));
		SmartDashboard.putData("Autonomous Move", chooser);

	}

	public void commonPeriodic() {
		// SmartDashboard.putNumber("PowerDistributionTemperature",
		// powerDistribution.getTemperature());
		SmartDashboard.putNumber(
				"PowerDistribution Total Motor Current",
				powerDistribution.getCurrent(12)
						+ powerDistribution.getCurrent(13)
						+ powerDistribution.getCurrent(14)
						+ powerDistribution.getCurrent(15));
		// SmartDashboard.putNumber("PowerDistribution Back Right Motor Current",
		// powerDistribution.getCurrent(12));

		// SmartDashboard.putNumber("PowerDistribution Front Right Motor Current",
		// powerDistribution.getCurrent(13));
		// SmartDashboard.putNumber("PowerDistribution Back Left Motor Current",
		// powerDistribution.getCurrent(14));
		// SmartDashboard.putNumber("PowerDistribution Front Left Motor Current",
		// powerDistribution.getCurrent(15));

		// Put accelerations and positions
		SmartDashboard.putNumber("Current Z Acceleration",
				accel.getZAcceleration() - zAccelMean);
		SmartDashboard.putNumber("Current X Displacement",
				xDisplacement.mDisplacementIntegral);
		SmartDashboard.putNumber("Current Y Displacement",
				yDisplacement.mDisplacementIntegral);
		SmartDashboard.putNumber("Current Z Displacement",
				zDisplacement.mDisplacementIntegral);
		// SmartDashboard.putNumber("Angular Acceleration", gyro.getRate());
		// SmartDashboard.putNumber("Angular Positon", gyro.getAngle());
		SmartDashboard.putNumber("Arm Encoder", armEncoder.getDistance());
		SmartDashboard.putNumber("Totem Encoder", fingerEncoder.getDistance());
		SmartDashboard.putBoolean("Arm Limit", armLimitSwitch.get());
		SmartDashboard.putBoolean("Finger Limit", topFingerLimitSwitch.get());
		SmartDashboard.putBoolean("Mid High Finger Limit",
				midHighFingerLimitSwitch.get());
		SmartDashboard.putBoolean("Mid Low Finger Limit",
				midLowFingerLimitSwitch.get());
		SmartDashboard.putBoolean("Bottem Finger Limit",
				lowFingerLimitSwitch.get());
		SmartDashboard
				.putBoolean(
						"In Between Limit Switches",
						!(topFingerLimitSwitch.get()
								&& lowFingerLimitSwitch.get()
								&& midLowFingerLimitSwitch.get() && midHighFingerLimitSwitch
								.get()));
		SmartDashboard.putNumber("Totem Current",
				powerDistribution.getCurrent(7));

		// SmartDashboard.putNumber("Chassis X", chasis.getX());

		SmartDashboard
				.putBoolean("Limit Fish",
						(armEncoder.getDistance() < 0.75 || armEncoder
								.getDistance() > 1.5));

		SmartDashboard.putNumber("Front Left IR Sensor",
				RobotConstants.sensorVoltageToCm(forwardLeft.getVoltage()));
		SmartDashboard.putNumber("Front Right IR Sensor",
				RobotConstants.sensorVoltageToCm(forwardRight.getVoltage()));
		SmartDashboard.putNumber("Front Middle IR Sensor",
				RobotConstants.sensorVoltageToCm(forwardMiddle.getVoltage()));
		SmartDashboard
				.putNumber("Front Offset IR Sensor", RobotConstants
						.sensorVoltageToCm(forwardOffCenter.getVoltage()));

		SmartDashboard
				.putBoolean(
						"Straight?",
						Math.abs(RobotConstants.sensorVoltageToCm(forwardLeft
								.getVoltage())
								- RobotConstants.sensorVoltageToCm(forwardRight
										.getVoltage())) < RobotConstants.kStraightAlignmentDeadband);
		SmartDashboard
				.putBoolean(
						"Centered?",
						(RobotConstants.sensorVoltageToCm(forwardMiddle
								.getVoltage()) < RobotConstants.kObjectpLength)
								&& (RobotConstants
										.sensorVoltageToCm(forwardOffCenter
												.getVoltage()) < RobotConstants.kObjectpLength));

		// Listen to arguments
		double leftMotorCurrent = powerDistribution
				.getCurrent(RobotConstants.kLeftArmMotorCurrent);
		double rightMotorCurrent = powerDistribution
				.getCurrent(RobotConstants.kRightArmMotorCurrent);

		SmartDashboard
				.putBoolean(
						"Arm Motors' Fighting",
						!(Math.abs(leftMotorCurrent - rightMotorCurrent) > RobotConstants.kArmMotorDifferenceTolerance));
		SmartDashboard.putNumber("Arm Motors' Disagreement Measure",
				Math.abs(leftMotorCurrent - rightMotorCurrent));

		if (!this.isDisabled()) {
			armController.periodic();
			if (weapons.getThrottle() < 0) {
				// fingerController.periodic();
			}
		}

		changeDriveStyle.update();
		overrideTracker.update();

		blue1.update();
		blue2.update();
		blue3.update();
		blue4.update();
		blue5.update();
		blue6.update();
		triggerButton.update();
		smallUp.update();
		smallDown.update();
		largeUp.update();
		largeDown.update();
		upTurnSpeed.update();
		antennieButton.update();
		grabberRotateButton.update();
		toteUp.update();
		toteDown.update();

		if (!armLimitSwitch.get()) {
			armEncoder.setOffset(RobotConstants.kArmPositionZenith);
		}

		if (!topFingerLimitSwitch.get()) {
			if (fingerEncoder.getRate() > 0.05) {
				fingerDangerousTerritory = true;
			} else if (fingerEncoder.getRate() < 0.05) {
				fingerDangerousTerritory = false;
			}

			downfulnessTimeOut.reset();
			downfulnessTimeOut.start();
			fingerEncoder.setPosition(42.5); // Inches
		}

		if (!lowFingerLimitSwitch.get()) { // This is what is going to go
											// horribly wrong.
			fingerEncoder.setPosition(9);
		}

		if (!midLowFingerLimitSwitch.get()) {
			fingerEncoder.setPosition(24);
		}

		if (!midHighFingerLimitSwitch.get()) {
			fingerEncoder.setPosition(39);
		}

		if (fingerEncoder.getDistance() < 0 && false) {
			if (fingerEncoder.getRate() < 0.05) {
				fingersAtBottom = true;
			} else if (fingerEncoder.getRate() > 0.05) {
				fingersAtBottom = false;
			}

			upfulnessTimeOut.reset();
			upfulnessTimeOut.start();
		}
	}

	public void autonomousInit() {
		xAccelMean = mean(xAccelCalibration);
		yAccelMean = mean(yAccelCalibration);
		zAccelMean = mean(zAccelCalibration);

		armEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);

		armController.init();
		fingerController.init();

		if (autoMove == null) {
			autoMove = new GrabStepCanPutFront(this);
		}
		// autoMove = (AutoMove) chooser.getSelected();
		autoMove.init();

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		// System.out.println(accel.getXAcceleration() + "," +
		// accel.getYAcceleration() + "," + accel.getZAcceleration());
		commonPeriodic();

		if (!autoStopped) {
			Status status = autoMove.periodic();
			if (status == Status.isNotAbleToContinue
					|| status == Status.isAbleToContinue
					|| status == Status.emergency) {
				autoStopped = true;
			}
		}
	}

	public void disabledPeriodic() {
		AutoMove selected = (AutoMove) chooser.getSelected();
		if (selected != null) {
			lastSeenAutoMove = selected;
			autoMove = selected;
		} else if (lastSeenAutoMove != null) {
			autoMove = lastSeenAutoMove;
		} else {
			autoMove = new GrabStepCanPutFront(this);
		}

		// System.out.println(accel.getXAcceleration() + "," +
		// accel.getYAcceleration() + "," + accel.getZAcceleration());
		xAccelCalibration = shift(xAccelCalibration);
		yAccelCalibration = shift(yAccelCalibration);
		zAccelCalibration = shift(zAccelCalibration);

		xAccelCalibration[0] = accel.getXAcceleration();
		yAccelCalibration[0] = accel.getYAcceleration();
		zAccelCalibration[0] = accel.getZAcceleration();
		// xGyroCalibration = gyro.getRate();
		// timeLag.start();
		// System.out.println("Entered Teleop");
		// System.out.println(timeOut.get());
		// Put currents and temperature on the smartDashboard
		// System.out.println("Telleop begins" + timeLag.get());
		commonPeriodic();
	}

	@Override
	public void teleopInit() {
		armEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
		startedMovingTimeOut.reset();
		startedMovingTimeOut.start();

		armMotors.manual = false;
		armController.enabled = true;
		fingerController.enabled = true;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		if (weapons.getRawButton(5) || chasis.getRawButton(5)) {
			startedMovingTimeOut.reset();
			startedMovingTimeOut.start();
			movingIndependantly = MovingState.up;
		}
		if (weapons.getRawButton(10) || chasis.getRawButton(10)) {
			startedMovingTimeOut.reset();
			startedMovingTimeOut.start();
			movingIndependantly = MovingState.down;
		}

		if ((!(topFingerLimitSwitch.get() && lowFingerLimitSwitch.get()
				&& midLowFingerLimitSwitch.get() && midHighFingerLimitSwitch
					.get()) || Math.abs(weapons.getTwist()) > 0.1)
				&& startedMovingTimeOut.get() > 0.2) {
			movingIndependantly = MovingState.still;
		}

		if (armEncoder.getRate() > 0) {
			armForwards = true;
		} else if (armEncoder.getRate() < 0) {
			armBackwards = true;
		} else {
			armForwards = false;
			armBackwards = false;
		}

		if (blue1.justPressedp()) {
			fingerController.setSetpoint(RobotConstants.kFingerSetpoints[1]);
		} else if (blue2.justPressedp()) {
			fingerController.setSetpoint(RobotConstants.kFingerSetpoints[2]);
		} else if (blue3.justPressedp()) {
			fingerController.setSetpoint(RobotConstants.kFingerSetpoints[3]);
		} else if (blue4.justPressedp()) {
			fingerController.setSetpoint(RobotConstants.kFingerSetpoints[4]);
		} else if (blue5.justPressedp()) {
			fingerController.setSetpoint(RobotConstants.kFingerSetpoints[5]);
		} else if (blue6.justPressedp()) {
			fingerController.setSetpoint(RobotConstants.kFingerSetpoints[6]);
		} else if (weapons.getPOV() != -1) {
			fingerController.setSetpoint(RobotConstants.kFingerSetpoints[0]);
		}

		// fingerController.enabled = false;
		if (fingerDangerousTerritory && downfulnessTimeOut.get() < 2) {
			realFingerMotor.set(-0.15);
		} else if (fingersAtBottom && upfulnessTimeOut.get() < 1) {
			realFingerMotor.set(0.15);
		} else if (movingIndependantly == MovingState.up) {
			realFingerMotor.set(0.5);
		} else if (movingIndependantly == MovingState.down) {
			realFingerMotor.set(-0.5);
		} else if (weapons.getThrottle() > 0) {
			realFingerMotor.set(weapons.getTwist()
					* Math.abs(weapons.getTwist()));
		} else {
			realFingerMotor.set(0);
			// fingerController.periodic();
		}

		// Drive style determines weather left and right are turn or strafe.

		/*
		 * if (changeDriveStyle.justPressedp()) { driveStyle = !driveStyle; }
		 */

		// Limits on arm positions

		if ((fingerEncoder.getDistance() > 20 & armForwards & !armLimitSwitch
				.get())) {
			armMotors.setMaxSpeed(-0.1);
			bouncyTimeOut.reset();
			bouncyTimeOut.start();
			hasHitBouncy = true;
		} else if (bouncyTimeOut.get() < 1 && hasHitBouncy) {
			armMotors.setMaxSpeed(-0.1);
		} else if (armEncoder.getDistance() > RobotConstants.kArmPositionBehind
				&& blue2.Pressedp()) {
			armMotors.setMaxSpeed(-0.1);
		} else if (armEncoder.getDistance() < RobotConstants.kArmPositionGrab
				&& blue2.Pressedp()) {
			armMotors.setMinSpeed(0.1);
		} else {
			armMotors
					.setMaxSpeed(triggerButton.Pressedp() ? RobotConstants.kArmLimitedSpeed
							: 1.0);
			armMotors
					.setMinSpeed(triggerButton.Pressedp() ? -RobotConstants.kArmLimitedSpeed
							: -1.0);
		}

		armController.setSetpoint(-weapons.getY() * 10);

		// Temporary variables
		double x, y, rotate, turned, sensitivity;
		turned = 0; //((gyro.getAngle()));// / 180.0) * Math.PI);
		sensitivity = (chasis.getAxis(Joystick.AxisType.kThrottle) * -1 + 1) * 0.9 + 0.1;

		if (upTurnSpeed.Pressedp()) {
			turnSpeed = true;
		} else {
			turnSpeed = false;
		}

		if (driveStyle) {
			y = -chasis.getAxis(Joystick.AxisType.kZ);
			x = -chasis.getAxis(Joystick.AxisType.kY);
			rotate = -chasis.getAxis(Joystick.AxisType.kX);
		} else {
			y = -chasis.getAxis(Joystick.AxisType.kX);
			x = -chasis.getAxis(Joystick.AxisType.kY);
			rotate = -chasis.getAxis(Joystick.AxisType.kZ);
		}

		// System.out.println("The Joystick Rotation: " + rotate);

		x *= sensitivity;
		y *= sensitivity;
		if (turnSpeed) {
			rotate *= sensitivity;
		} else {
			rotate *= sensitivity * 0.2;
		}

		/*
		 * double rotationRate = gyro.getRate(); if (rotationRate < (rotate *
		 * RobotConstants.kMaxRotationSpeed + RobotConstants.kRotationTolerance)
		 * && rotationRate > (rotate * RobotConstants.kMaxRotationSpeed -
		 * RobotConstants.kRotationTolerance)) { rotate += (rotationRate -
		 * rotate); }
		 */

		// y += tipsyness.tipped();
		// x += swayfulness.tipped();

		// System.out.println("Field Centric " + fieldcentric + "\n Gyro " +
		// gyro.getAngle());
		// Deadbanding
		if (Math.abs(x) < RobotConstants.kDeadband) {
			x = 0;
		}

		if (Math.abs(y) < RobotConstants.kDeadband) {
			y = 0;
		}

		if (Math.abs(rotate) < (RobotConstants.kDeadband)) {
			rotate = 0;
		}

		if ((chasis.getRawButton(RobotConstants.kAutoAlignButton) || weapons.getRawButton(RobotConstants.kAutoAlignButton))) {
//			boolean slamIt = chasis.getRawButton(RobotConstants.kAutoAlignSlamButton) || weapons.getRawButton(RobotConstants.kAutoAlignSlamButton);
			boolean slamIt = false;
			double[] newDriveValues = autoAlign(x, y, rotate, slamIt);
			x = newDriveValues[0];
			y = newDriveValues[1];
			rotate = newDriveValues[2];
		}
		
		driveTrain.mecanumDrive_Cartesian(y, x, rotate, 0);

		// System.out.println("True Middle Teleop" + timeLag.get());

		// Manual gyro reseting
		if (chasis.getPOV() != -1) {
			gyro.set(chasis.getPOV());
		}

//		armPistons.set(triggerButton.Pressedp() || grippersLatched);
		armPistons.set(grippersLatched);

		if (triggerButton.justPressedp()) {
			grippersLatched = !grippersLatched;
		}

		if (stopSpin.Pressedp()) {
			gyro.resetRate();
		}

		// System.out.println("After Arm pistons" + timeLag.get());

		// Update button tracker

		if (antennieButton.Pressedp()) {
			antennie.set(Value.kForward);
		} else {
			antennie.set(Value.kReverse);
		}

		if (overrideTracker.Pressedp()) {
			armMotors.jamesBond(weapons.getY() * -0.5);
		} else {

			armController.periodic();
		}

		if (grabberRotateButton.Pressedp()
		/* (armEncoder.getDistance() < 0.75 || armEncoder.getDistance() > 1.5) */) {
			grabberRotatePiston.set(Value.kForward);
		} else {
			grabberRotatePiston.set(Value.kReverse);
		}

		// System.out.println("Telleop Ends" + timeLag.get());

		// System.out.println("After Button updates" + timeLag.get());

		commonPeriodic();

	}

	public double[] autoAlign(double x, double y, double rotate, boolean slamIt) {
		// AutoCentering
		double left = RobotConstants.sensorVoltageToCm(forwardLeft.getVoltage());
		double right = RobotConstants.sensorVoltageToCm(forwardRight.getVoltage());
		double center = RobotConstants.sensorVoltageToCm(forwardMiddle.getVoltage());
		double justRightOfCenter = RobotConstants.sensorVoltageToCm(forwardOffCenter.getVoltage());
		
		// Floor the sensors to min range
		left = Math.max(0, left);
		right = Math.max(0, right);
		center = Math.max(0, center);
		justRightOfCenter = Math.max(0, justRightOfCenter);
		
		final double maxSpinSpeed = 0.5;
		final double minSpinSpeed = 0.15;
		final double topSpinSpeedAngle = 20.0;
		final double crabSpeed = 0.2;
		final double approachSpeed = slamIt? 0.5 : 0.3;
		final double thereRange = 3;
		final double maxCenteringRange = 10; // Above a certain range, we can't see down canyons, don't even try to align
		final double minRotatingRange = 3; // Closer than this, you'll do more harm than good by turning 
		final double maxRotatingRange = 30; // Farther than this, the sensor reading isn't accurate enough 
		final double canyonRangeDelta = 4;  // surfaces farther than this from the range are considered canyon

		// Determine approach speed and rotation speed from left and right sensors
		double robotRange = (left + right) / 2.0;
		double robotAngle = (right - left); // not really an angle; we're just using it like it is
		if (Math.abs(robotAngle) > RobotConstants.kStraightAlignmentDeadband
				&& left < maxRotatingRange && right < maxRotatingRange) {
			System.out.printf("    ");
			// P control
			rotate = (Math.abs(robotAngle) / topSpinSpeedAngle) * maxSpinSpeed;
			rotate = Math.max(minSpinSpeed, rotate);
			rotate = Math.min(maxSpinSpeed, rotate);
			rotate = (robotAngle > 0? 1.0:-1.0) * rotate;
//			rotate = (robotAngle > 0? 1:-1) * topSpinSpeed; // bang bang control
		} else {
			System.out.printf("not ");
			rotate = 0;
		} 
//		rotate = 0;
		System.out.printf("rotating. ");
		x = (robotRange > thereRange)? approachSpeed:0;
		System.out.printf((robotRange > thereRange)? "    approaching. ": "not approaching. ");

		
		// Determine crab direction from center and justOffCenter sensors
		if(robotRange > maxCenteringRange) {
			// We're far enough away that we can't reliably detect the canyon
			System.out.printf("too far to center.  ");
			y = 0;
		} else if (center - robotRange > canyonRangeDelta) {
			System.out.printf("centered!           ");
			y = 0;
		} else if (justRightOfCenter - robotRange > canyonRangeDelta) {
			System.out.printf("jroc down canyon.   ");
			y = -crabSpeed;
		} else {
			// Chances are the canyon is to the left of center
			System.out.printf("nothin down canyon. ");
			y = crabSpeed;
		}
		
		System.out.printf("got: left: %f right: %f center: %f jroc: %f x:%f y:%f r:%f\n",
				left, right, center, justRightOfCenter, x, y, rotate);
		
		double[] retval = {x, y, rotate}; 
//		double[] retval = {0, 0, 0}; 
		return retval;
	}

	public double reccomendedSpeed() {
		System.out.println("Reccomendation made.");
		double theta1 = tipsyness.tipped();
		double theta2 = armEncoder.getDistance();
		if (theta2 > Math.PI / 4) {
			double inside = Math.sin(Math.PI / 4 - theta1 + theta2);
			double outside = 1 / Math.tan(Math.PI - theta1);
			return armEncoder.getRate()
					- ((85 - 43.25 * inside - 17.3 * inside * outside) / ((5 * inside - 2
							* inside * outside) * 8.65 * 5)) + 0.01;
		} else {
			return armEncoder.getRate() + 0.39306 / Math.cos(theta1) - 1 + 0.01;
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {

	}

	private double[] shift(double[] table) {
		for (int i = table.length - 1; i > 0; i--) {
			table[i] = table[i - 1];
		}
		return table;
	}

	private double mean(double[] table) {
		double sum = 0;
		for (int i = 0; i < table.length; i++) {
			sum += table[i];
		}
		return sum / table.length;
	}

}

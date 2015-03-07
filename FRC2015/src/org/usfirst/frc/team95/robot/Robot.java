/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team95.robot;

import org.usfirst.frc.team95.robot.auto.AntennieAndTotes;
import org.usfirst.frc.team95.robot.auto.AntennieGrabAndBack;
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
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
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

	ButtonTracker changeDriveStyle, rotate90Left, rotate90Right, autoStack,
			overrideTracker, blue1, blue2, blue3, blue4, blue5, blue6,
			triggerButton, stopSpin, upTurnSpeed, antennieButton,
			grabberRotateButton;

	boolean driveStyle, rotating, fingersAtBottom, fieldcentric = false;
	boolean armForwards = false;
	boolean armBackwards = false;
	double targetAngle;

	Timer timeOut;
	Timer timeLag;

	PowerDistributionPanel powerDistribution;

	double[] xAccelCalibration, yAccelCalibration, zAccelCalibration,
			xGyroCalibration, yGyroCalibration, zGyroCalibration;

	double xAccelMean, yAccelMean, zAccelMean, xGyroMean, yGyroMean, zGyroMean;

	public FauxPID armController, fingerController;

	public DoubleSolenoid armPistons, antennie, grabberRotatePiston;

	AutoMove autoMove;

	SendableChooser chooser;
	private boolean autoStopped, fingerDangerousTerritory, hasHitBouncy;

	Compressor compressor;

	TippynessMeasure tipsyness, swayfulness;

	public DigitalInput armLimitSwitch, topFingerLimitSwitch, lowFingerLimitSwitch, midLowFingerLimitSwitch, midHighFingerLimitSwitch;
	private MotorWrapper realFingerMotor;

	Timer bouncyTimeOut, downfulnessTimeOut, upfulnessTimeOut;

	ButtonTracker smallUp, smallDown, largeUp, largeDown;

	boolean grippersLatched = false;
	
	AutoMove lastSeenAutoMove = null;
	
	//false is slow turn
	boolean turnSpeed = false;
	

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

		gyro = new ResetableGyro(RobotConstants.kGyro);
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
		antennieButton = new ButtonTracker(weapons, RobotConstants.kAntennieButton);
		grabberRotateButton = new ButtonTracker(weapons, RobotConstants.kGrabberRotateButton);
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

		armPistons = new DoubleSolenoid(RobotConstants.kPCMId,
				RobotConstants.kArmPistons, RobotConstants.kArmPistons + 1);

		compressor = new Compressor();
		compressor.start();

		chooser = new SendableChooser();
		chooser.addObject("Zombie", new NoMove(this));
		// chooser.addObject("TakeToteRight", new TakeToteRight(this));
		// chooser.addObject("TakeGoldenTotes", new GrabGoldenTotes(this));
		// chooser.addObject("Dance", new Dance(this));
		// chooser.addObject("GrabMaximumFrontAndStack",
		// new GrabMaximumFrontAndStack(this));
		chooser.addObject("Grab Barrier Can and Move Back",
				new GrabLeftCentralCan(this));
		// chooser.addObject("Move the Arm", new PlainMotorMove(armMotors, 0.25,
		// 1.0));
		// chooser.addObject("Floor Can", new GrabCanFromFloor(this));
		chooser.addObject("Move Forward", new GoForward(this));
		 chooser.addObject("Grab Can and Move Left", new
		 GrabFrontMoveLeft(this));
		chooser.addObject("Grab Can and Move Right", new GrabFrontMoveRight(
				this));
		// chooser.addObject("Grab Can and Move Forward", new
		// GrabFrontMoveForwards(this));
		chooser.addObject("Grab Can and Move Backward",
				new GrabFrontMoveBackwards(this));
		//chooser.addObject("Grab Barrier Can and Place Behind",
		//		new GrabStepCanPutBehind(this));
		chooser.addObject("Grab Barrier Can and Place Front",
				new GrabStepCanPutFront(this));
		chooser.addObject("Quick Barrier Grab", new QuickBarrierGrab(this));
		chooser.addObject("Plain Antennie", new PlainAntennieGrab(this));
		chooser.addObject("Antennie And Totes", new AntennieAndTotes(this));
		chooser.addObject("Antennie And Move Back", new AntennieGrabAndBack(this));
		SmartDashboard.putData("Autonomous Move", chooser);
		
		

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

		smallUp = new ButtonTracker(chasis, 5);
		smallDown = new ButtonTracker(chasis, 10);
		largeUp = new ButtonTracker(chasis, 6);
		largeDown = new ButtonTracker(chasis, 9);
		
		antennie = new DoubleSolenoid(RobotConstants.kPCMId, RobotConstants.kAntennie, RobotConstants.kAntennie + 1);
		grabberRotatePiston = new DoubleSolenoid(RobotConstants.kPCMId, RobotConstants.kGrabberRotatePiston, RobotConstants.kGrabberRotatePiston + 1);
	}

	public void commonPeriodic() {
		SmartDashboard.putNumber("PowerDistributionTemperature",
				powerDistribution.getTemperature());
		SmartDashboard.putNumber(
				"PowerDistribution Total Motor Current",
				powerDistribution.getCurrent(12)
						+ powerDistribution.getCurrent(13)
						+ powerDistribution.getCurrent(14)
						+ powerDistribution.getCurrent(15));
		SmartDashboard.putNumber("PowerDistribution Back Right Motor Current",
				powerDistribution.getCurrent(12));

		SmartDashboard.putNumber("PowerDistribution Front Right Motor Current",
				powerDistribution.getCurrent(13));
		SmartDashboard.putNumber("PowerDistribution Back Left Motor Current",
				powerDistribution.getCurrent(14));
		SmartDashboard.putNumber("PowerDistribution Front Left Motor Current",
				powerDistribution.getCurrent(15));

		// Put accelerations and positions
		SmartDashboard.putNumber("Current Z Acceleration",
				accel.getZAcceleration() - zAccelMean);
		SmartDashboard.putNumber("Current X Displacement",
				xDisplacement.mDisplacementIntegral);
		SmartDashboard.putNumber("Current Y Displacement",
				yDisplacement.mDisplacementIntegral);
		SmartDashboard.putNumber("Current Z Displacement",
				zDisplacement.mDisplacementIntegral);
		SmartDashboard.putNumber("Angular Acceleration", gyro.getRate());
		SmartDashboard.putNumber("Angular Positon", gyro.getAngle());
		SmartDashboard.putNumber("Arm Encoder", armEncoder.getDistance());
		SmartDashboard.putNumber("Totem Encoder", fingerEncoder.getDistance());
		SmartDashboard.putBoolean("Arm Limit", armLimitSwitch.get());
		SmartDashboard.putBoolean("Finger Limit", topFingerLimitSwitch.get());
		SmartDashboard.putBoolean("Bottem Finger Limit", lowFingerLimitSwitch.get());
		SmartDashboard.putNumber("Totem Current",
				powerDistribution.getCurrent(7));

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
				//fingerController.periodic();
			}
		}

		changeDriveStyle.update();
		rotate90Right.update();
		rotate90Left.update();
		overrideTracker.update();

		blue1.update();
		blue2.update();
		blue3.update();
		blue4.update();
		blue5.update();
		blue6.update();
		autoStack.update();
		triggerButton.update();
		smallUp.update();
		smallDown.update();
		largeUp.update();
		largeDown.update();
		upTurnSpeed.update();
		antennieButton.update();
		grabberRotateButton.update();

		if (!armLimitSwitch.get()) {
			if (armEncoder.getRate() > 0) {
				armEncoder.setOffset(RobotConstants.kArmPositionZenith
						- RobotConstants.kArmLimitSwitchSloppyness
						- armEncoder.getDistance());
			} else {
				armEncoder.setOffset(RobotConstants.kArmPositionZenith
						+ RobotConstants.kArmLimitSwitchSloppyness
						- armEncoder.getDistance());
			}
		}

		if (!topFingerLimitSwitch.get()) {
			if (fingerEncoder.getRate() > 0.05) {
				fingerDangerousTerritory = true;
			} else if (fingerEncoder.getRate() < 0.05) {
				fingerDangerousTerritory = false;
			}

			downfulnessTimeOut.reset();
			downfulnessTimeOut.start();
			fingerEncoder.setPosition(43); // Inches
		}
		
		if(!lowFingerLimitSwitch.get()) { // This is what is going to go horribly wrong.
			if (fingerEncoder.getRate() < 0.05) {
				fingersAtBottom = true;
			} else if (fingerEncoder.getRate() > 0.05) {
				fingersAtBottom = false;
			}
			
			upfulnessTimeOut.reset();
			upfulnessTimeOut.start();
			fingerEncoder.setPosition(8);
		}
		
		if(!midLowFingerLimitSwitch.get()) {
			fingerEncoder.setPosition(23);
		}
		
		if(!midHighFingerLimitSwitch.get()) {
			fingerEncoder.setPosition(38);
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
		//autoMove = (AutoMove) chooser.getSelected();
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

		armMotors.manual = false;
		armController.enabled = true;
		fingerController.enabled = true;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		// Put currents and temperature on the smartDashboard

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
		} else if (weapons.getThrottle() > 0) {
			realFingerMotor.set(weapons.getTwist()
					* Math.abs(weapons.getTwist()));
		} else {
			realFingerMotor.set(0);
			//fingerController.periodic();
		}

		// Drive style determines weather left and right are turn or strafe.

		if (changeDriveStyle.justPressedp()) {
			driveStyle = !driveStyle;
		}

		// Limits on arm positions

		if ((fingerEncoder.getDistance() > 3 & armForwards & !armLimitSwitch
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
		turned = ((gyro.getAngle()));// / 180.0) * Math.PI);
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
		if (turnSpeed = true) {
			rotate *= sensitivity;
		}else {
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

		// System.out.println("True Middle Teleop" + timeLag.get());

		// Manual gyro reseting
		if (chasis.getPOV() != -1) {
			gyro.set(chasis.getPOV());
		}

		if (triggerButton.Pressedp() || grippersLatched) {
			armPistons.set(Value.kForward);
		} else {
			armPistons.set(Value.kReverse);
		}

		if (triggerButton.Pressedp()) {
			grippersLatched = false;
		}

		if (stopSpin.Pressedp()) {
			gyro.resetRate();
		}

		// System.out.println("After Arm pistons" + timeLag.get());

		// Update button tracker

		if (antennieButton.Pressedp() && 
				(armEncoder.getDistance() < 0.95993 || armEncoder.getDistance() > 2.18166)) { // 55 and 125 degrees
			antennie.set(Value.kForward);
		} else {
			antennie.set(Value.kReverse);
		}
		
		if (overrideTracker.Pressedp()) {
			armMotors.jamesBond(weapons.getY() * -0.5);
		} else {

			armController.periodic();
		}

		if (grabberRotateButton.Pressedp()) {
			grabberRotatePiston.set(Value.kForward);
		} else {
			grabberRotatePiston.set(Value.kReverse);
		}
		
		// System.out.println("Telleop Ends" + timeLag.get());

		// System.out.println("After Button updates" + timeLag.get());

		commonPeriodic();

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

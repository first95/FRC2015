/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team95.robot;


import org.usfirst.frc.team95.robot.auto.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	/*
	 * This is the definitions section, where all variables
	 * that need to persist are declared to exist.
	 */
	
    Talon frontLeft, frontRight, backLeft, backRight, armTalon, fingerTalon;
    public MotorWrapper realFrontLeft, realFrontRight, realBackLeft, realBackRight;
    public Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder, armEncoder, fingerEncoder;
    public RobotDrive driveTrain;
    
    public ResetableGyro gyro;
    ADXL345_I2C extraAccel;
    BuiltInAccelerometer builtinAccel;
    public CommonFilter accel;
    
    PositionTracker xDisplacement, yDisplacement, zDisplacement;
    
    Joystick chasis, weapons;
    
    ButtonTracker changeDriveStyle, rotate90Left, rotate90Right, fieldCentric, blue1, blue2, blue3, blue4, blue5, blue6;
    boolean driveStyle, rotating, fieldcentric;
    double targetAngle;
    
    Timer timeOut;
    
    PowerDistributionPanel powerDistribution;
    
    double[] xAccelCalibration, yAccelCalibration, zAccelCalibration, xGyroCalibration, yGyroCalibration, 
    		zGyroCalibration;
    
    double xAccelMean, yAccelMean, zAccelMean, xGyroMean, yGyroMean, zGyroMean;
    
    public PIDController armController, fingerController;
    
    public Solenoid armPistons;
    
    AutoMove autoMove;
    
    SendableChooser chooser;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	/*
    	 * This sets up everything's values.
    	 */
        frontLeft = new Talon(RobotConstants.kFrontLeftMotor);
        frontRight = new Talon(RobotConstants.kFrontRightMotor);
        backLeft = new Talon(RobotConstants.kBackLeftMotor);
        backRight = new Talon(RobotConstants.kBackRightMotor);
        armTalon = new Talon(RobotConstants.kArmMotor);
        fingerTalon = new Talon(RobotConstants.kFingerMotor);
        realFrontLeft = new MotorWrapper(frontLeft);
        realFrontRight = new MotorWrapper(frontRight);
        realBackLeft = new MotorWrapper(backLeft);
        realBackRight = new MotorWrapper(backRight);
        frontLeftEncoder = new Encoder(RobotConstants.kFrontLeftEncoder, RobotConstants.kFrontLeftEncoder + 1);
        backLeftEncoder = new Encoder(RobotConstants.kBackLeftEncoder, RobotConstants.kBackLeftEncoder + 1);
        frontRightEncoder = new Encoder(RobotConstants.kFrontRightEncoder, RobotConstants.kFrontRightEncoder + 1);
        backRightEncoder = new Encoder(RobotConstants.kBackRightEncoder, RobotConstants.kBackRightEncoder + 1);
        armEncoder = new Encoder(RobotConstants.kArmEncoder, RobotConstants.kArmEncoder + 1);
        armEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        fingerEncoder = new Encoder(RobotConstants.kFingerEncoder, RobotConstants.kFingerEncoder + 1);
        fingerEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        driveTrain = new RobotDrive(realFrontLeft, realBackLeft, realFrontRight, realBackRight);
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

        changeDriveStyle = new ButtonTracker(chasis, RobotConstants.kChangeDriveStyle);
        fieldCentric = new ButtonTracker(chasis, RobotConstants.kFieldCentric);
        driveStyle = false; // False == traditional
        rotate90Left = new ButtonTracker(chasis, RobotConstants.kRotate90Left);
        rotate90Right = new ButtonTracker(chasis, RobotConstants.kRotate90Right);
        blue1 = new ButtonTracker(weapons, 1);
        blue2 = new ButtonTracker(weapons, 2);
        blue3 = new ButtonTracker(weapons, 3);
        blue4 = new ButtonTracker(weapons, 4);
        blue5 = new ButtonTracker(weapons, 5);
        blue6 = new ButtonTracker(weapons, 6);
        rotating = false;
        fieldcentric = true;
        
        timeOut = new Timer();
        powerDistribution = new PowerDistributionPanel();
        
        xAccelCalibration = new double[RobotConstants.kCalibrationLength]; 
        yAccelCalibration = new double[RobotConstants.kCalibrationLength]; 
        zAccelCalibration = new double[RobotConstants.kCalibrationLength];
        
        armController = new PIDController(RobotConstants.kArmP, RobotConstants.kArmI, RobotConstants.kArmD, 
        		armEncoder, armTalon, RobotConstants.kPIDUpdateInterval);
        armController.setAbsoluteTolerance(RobotConstants.kArmTolerance);
        armController.enable();
        
        fingerController = new PIDController(RobotConstants.kFingerP, RobotConstants.kFingerI, 
        		RobotConstants.kFingerD, fingerEncoder, fingerTalon, RobotConstants.kPIDUpdateInterval);
        fingerController.setAbsoluteTolerance(RobotConstants.kFingerTolerance);
        fingerController.enable();
        
        armPistons = new Solenoid(RobotConstants.kArmPistons);
        
        chooser = new SendableChooser();
        chooser.addDefault("Zombie", new NoMove(this));
        chooser.addObject("TakeToteRight", new TakeToteRight(this));
        chooser.addObject("TakeGoldenTotes", new GrabGoldenTotes(this));
        SmartDashboard.putData("Autonomous Move", chooser);
    }
    
    public void autonomousInit() {
    	xAccelMean = mean(xAccelCalibration);
    	yAccelMean = mean(yAccelCalibration);
    	zAccelMean = mean(zAccelCalibration);
    	
    	armController.setSetpoint(0);
    	
    	autoMove = (AutoMove) chooser.getSelected();
    	autoMove.init();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	//System.out.println(accel.getXAcceleration() + "," + accel.getYAcceleration() + "," + accel.getZAcceleration());
    	autoMove.periodic();
    }
    
    public void disabledPeriodic() {
    	//System.out.println(accel.getXAcceleration() + "," + accel.getYAcceleration() + "," + accel.getZAcceleration());
    	xAccelCalibration = shift(xAccelCalibration);
    	yAccelCalibration = shift(yAccelCalibration);
    	zAccelCalibration = shift(zAccelCalibration);
    	
    	xAccelCalibration[0] = accel.getXAcceleration();
    	yAccelCalibration[0] = accel.getYAcceleration();
    	zAccelCalibration[0] = accel.getZAcceleration();
    	//xGyroCalibration = gyro.getRate();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	//System.out.println("Entered Teleop");
    	//System.out.println(timeOut.get());
    	// Put currents and temperature on the smartDashboard
    	SmartDashboard.putNumber("PowerDistributionTemperature", powerDistribution.getTemperature());
    	SmartDashboard.putNumber("PowerDistribution Total Motor Current", powerDistribution.getCurrent(12) + powerDistribution.getCurrent(13) + powerDistribution.getCurrent(14) + powerDistribution.getCurrent(15));
    	SmartDashboard.putNumber("PowerDistribution Back Right Motor Current", powerDistribution.getCurrent(12));
    	
    	SmartDashboard.putNumber("PowerDistribution Front Right Motor Current", powerDistribution.getCurrent(13));
    	SmartDashboard.putNumber("PowerDistribution Back Left Motor Current", powerDistribution.getCurrent(14));
    	SmartDashboard.putNumber("PowerDistribution Front Left Motor Current", powerDistribution.getCurrent(15));
    	
    	// Put accelerations and positions
    	SmartDashboard.putNumber("Current X Acceleration", accel.getXAcceleration() - xAccelMean);
    	SmartDashboard.putNumber("Current Y Acceleration", accel.getYAcceleration() - yAccelMean);
    	SmartDashboard.putNumber("Current Z Acceleration", accel.getZAcceleration() - zAccelMean);
    	SmartDashboard.putNumber("Current X Displacement", xDisplacement.mDisplacementIntegral);
    	SmartDashboard.putNumber("Current Y Displacement", yDisplacement.mDisplacementIntegral);
    	SmartDashboard.putNumber("Current Z Displacement", zDisplacement.mDisplacementIntegral);
    	SmartDashboard.putNumber("Angular Acceleration", gyro.getRate());
    	SmartDashboard.putNumber("Angular Positon", gyro.getAngle());
    	//System.out.println(timeOut.get());
    	
    	armController.setSetpoint(weapons.getX());
    	if (blue1.justPressedp()) {
    		fingerController.setSetpoint(RobotConstants.kFingerSetpoints[0]);
    	} else if (blue1.justPressedp()) {
    		fingerController.setSetpoint(RobotConstants.kFingerSetpoints[1]);
    	} else if (blue1.justPressedp()) {
    		fingerController.setSetpoint(RobotConstants.kFingerSetpoints[2]);
    	} else if (blue1.justPressedp()) {
    		fingerController.setSetpoint(RobotConstants.kFingerSetpoints[3]);
    	} else if (blue1.justPressedp()) {
    		fingerController.setSetpoint(RobotConstants.kFingerSetpoints[4]);
    	} else if (blue1.justPressedp()) {
    		fingerController.setSetpoint(RobotConstants.kFingerSetpoints[5]);
    	}
    	
    	// Drive style determines weather left and right are turn or strafe.
        if (changeDriveStyle.justPressedp()) {
            driveStyle = !driveStyle;
        }
      
        
        // Temporary variables
        double x, y, rotate, turned, sensitivity;
        turned = (gyro.getAngle() / 180.0 * Math.PI);
        sensitivity = (chasis.getAxis(Joystick.AxisType.kThrottle) * -1 + 1) * 0.9 + 0.1;
        
        if (driveStyle) {
            x = chasis.getAxis(Joystick.AxisType.kZ);
            y = -chasis.getAxis(Joystick.AxisType.kY);
            rotate = -chasis.getAxis(Joystick.AxisType.kX);
        } else {
            x = chasis.getAxis(Joystick.AxisType.kX);
            y = -chasis.getAxis(Joystick.AxisType.kY);
            rotate = -chasis.getAxis(Joystick.AxisType.kZ);
        }
        
        x *= sensitivity;
        y *= sensitivity;
        rotate *= sensitivity;
        
        // Deadbanding
        if (Math.abs(x) < RobotConstants.kDeadband) {
            x = 0;
        }
        
        if (Math.abs(y) < RobotConstants.kDeadband) {
            y = 0;
        }
        
        if (Math.abs(rotate) < RobotConstants.kDeadband) {
            rotate = 0;
        }
        
        if (fieldCentric.justPressedp()) {
        	fieldcentric = !fieldcentric;
        }
        
        if (rotate90Right.justPressedp()) {
        	targetAngle = gyro.getAngle() - 90;
        	System.out.println(targetAngle);
        	rotating = true;
        	timeOut.start();
        }
        else if(rotate90Left.justPressedp()){
        	targetAngle = gyro.getAngle() + 90;
        	System.out.println(targetAngle);
        	rotating = true;
        	timeOut.start();
        }
        
        // Turning stop condition
        if (Math.abs(gyro.getAngle() - targetAngle) < RobotConstants.kTurningCloseness ||
        		timeOut.get() > RobotConstants.kTurningTimeoute) {
        	rotating = false;
        	timeOut.stop();
        	timeOut.reset();
        }
        
        if (rotating) {
        	double turnSpeed = 0;
        	if ((targetAngle - turned) > 0) {
        		turnSpeed = 1;
        	} else {
        		turnSpeed = -1;
        	}
        	
        	if (fieldcentric) {
        		driveTrain.mecanumDrive_Cartesian(y, x, turnSpeed, turned);
        	} else {
        		driveTrain.mecanumDrive_Cartesian(y, x, turnSpeed, 0.0);
        	}
        } else {
        	if (fieldcentric) {
        		driveTrain.mecanumDrive_Cartesian(y, x, rotate, turned);
        	} else {
        		driveTrain.mecanumDrive_Cartesian(y, x, rotate, 0);
        	}
        }
        
        // Track acceleration.
        double accelX, accelY;
        accelX = Math.cos(turned) * (accel.getXAcceleration() - xAccelMean) +
                 Math.sin(turned) * (accel.getYAcceleration() - yAccelMean);
        accelY = Math.cos(turned) * (accel.getYAcceleration() - yAccelMean) +
                 Math.sin(turned) * (accel.getXAcceleration() - xAccelMean);
        xDisplacement.update(accelX);
        yDisplacement.update(accelY);
        zDisplacement.update(accel.getZAcceleration());
        
        //Manual gyro reseting
        if (chasis.getPOV() != -1) {
        	gyro.set(chasis.getPOV());
        }
        
        if (weapons.getRawButton(RobotConstants.kArmPistonsButton)) {
        	armPistons.set(true);
        } else {
        	armPistons.set(false);
        }
        	
        
        // Update button trackers
        changeDriveStyle.update();
        rotate90Right.update();
        rotate90Left.update();
        fieldCentric.update();
        
        blue1.update();
        blue2.update();
        blue3.update();
        blue4.update();
        blue5.update();
        blue6.update();
        
        
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

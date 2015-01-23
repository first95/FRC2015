/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team95.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
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
import edu.wpi.first.wpilibj.Gyro;
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
	
    Talon frontLeft, frontRight, backLeft, backRight, armTalon;
    MotorWrapper realFrontLeft, realFrontRight, realBackLeft, realBackRight;
    Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder, armEncoder;
    RobotDrive driveTrain;
    
    ResetableGyro gyro;
    ADXL345_I2C extraAccel;
    BuiltInAccelerometer builtinAccel;
    CommonFilter accel;
    
    PositionTracker xDisplacement, yDisplacement, zDisplacement;
    
    Joystick chasis, weapons;
    
    ButtonTracker changeDriveStyle, rotate90Left, rotate90Right, fieldCentric;
    boolean driveStyle, rotating, fieldcentric;
    double targetAngle;
    
    Timer timeOut;
    
    PowerDistributionPanel powerDistribution;
    
    double[] xAccelCalibration, yAccelCalibration, zAccelCalibration, xGyroCalibration, yGyroCalibration, 
    		zGyroCalibration;
    
    double xAccelMean, yAccelMean, zAccelMean, xGyroMean, yGyroMean, zGyroMean;
    
    PIDController armController;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	/*
    	 * This sets up everything's values.
    	 */
        frontLeft = new Talon(1);
        frontRight = new Talon(4);
        backLeft = new Talon(2);
        backRight = new Talon(3);
        armTalon = new Talon(5);
        realFrontLeft = new MotorWrapper(frontLeft);
        realFrontRight = new MotorWrapper(frontRight);
        realBackLeft = new MotorWrapper(backLeft);
        realBackRight = new MotorWrapper(backRight);
        frontLeftEncoder = new Encoder(2, 3);
        backLeftEncoder = new Encoder(4, 5);
        frontRightEncoder = new Encoder(6, 7);
        backRightEncoder = new Encoder(8, 9);
        armEncoder = new Encoder(1, 10);
        driveTrain = new RobotDrive(realFrontLeft, realBackLeft, realFrontRight, realBackRight);
        driveTrain.setInvertedMotor(MotorType.kFrontLeft, true);
        driveTrain.setInvertedMotor(MotorType.kRearLeft, true);
        
        gyro = new ResetableGyro(0);
        extraAccel = new ADXL345_I2C(Port.kOnboard, Accelerometer.Range.k8G);
        builtinAccel = new BuiltInAccelerometer();
        accel = new CommonFilter(extraAccel, builtinAccel);
        
        xDisplacement = new PositionTracker();
        yDisplacement = new PositionTracker();
        zDisplacement = new PositionTracker();
        
        chasis = new Joystick(0);
        weapons = new Joystick(1);

        changeDriveStyle = new ButtonTracker(chasis, 2);
        fieldCentric = new ButtonTracker(chasis, 5);
        driveStyle = false; // False == traditional
        rotate90Left = new ButtonTracker(chasis, 3);
        rotate90Right = new ButtonTracker(chasis, 4);
        rotating = false;
        fieldcentric = true;
        
        timeOut = new Timer();
        powerDistribution = new PowerDistributionPanel();
        
        xAccelCalibration = new double[500]; yAccelCalibration = new double[500]; zAccelCalibration = new double[500];
        xGyroCalibration = new double[500]; yGyroCalibration = new double[500]; zGyroCalibration = new double[500];
        
        armController = new PIDController(0.0, 0.0, 0.0, armEncoder, armTalon, 50.0);
        armController.enable();
    }
    
    public void autonomousInit() {
    	xAccelMean = mean(xAccelCalibration);
    	yAccelMean = mean(yAccelCalibration);
    	zAccelMean = mean(zAccelCalibration);
    	xGyroMean = mean(xGyroCalibration);
    	yGyroMean = mean(yGyroCalibration);
    	zGyroMean = mean(zGyroCalibration);
    	
    	armController.setSetpoint(0);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	//System.out.println(accel.getXAcceleration() + "," + accel.getYAcceleration() + "," + accel.getZAcceleration());

    }
    
    public void disabledPeriodic() {
    	//System.out.println(accel.getXAcceleration() + "," + accel.getYAcceleration() + "," + accel.getZAcceleration());
    	xAccelCalibration = shift(xAccelCalibration);
    	yAccelCalibration = shift(yAccelCalibration);
    	zAccelCalibration = shift(zAccelCalibration);
    	xGyroCalibration = shift(xGyroCalibration);
    	yGyroCalibration = shift(yGyroCalibration);
    	zGyroCalibration = shift(zGyroCalibration);
    	
    	xAccelCalibration[0] = accel.getXAcceleration();
    	yAccelCalibration[0] = accel.getYAcceleration();
    	zAccelCalibration[0] = accel.getZAcceleration();
    	//xGyroCalibration = gyro.getRate();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	System.out.println("Entered Teleop");
    	System.out.println(timeOut.get());
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
    	System.out.println(timeOut.get());
    	
    	// Drive style determines weather left and right are turn or strafe.
        if (changeDriveStyle.justPressedp()) {
            driveStyle = !driveStyle;
        }
        
        
        // Temporary variables
        double x, y, rotate, turned, sensitivity, temp;
        turned = (gyro.getAngle() / 180.0 * Math.PI);
        sensitivity = (chasis.getAxis(Joystick.AxisType.kThrottle) * -1 + 1) * 0.9 + 0.1;
        
        if (driveStyle) {
            x = chasis.getAxis(Joystick.AxisType.kZ);
            y = chasis.getAxis(Joystick.AxisType.kY);
            rotate = -chasis.getAxis(Joystick.AxisType.kX);
        } else {
            x = chasis.getAxis(Joystick.AxisType.kX);
            y = chasis.getAxis(Joystick.AxisType.kY);
            rotate = -chasis.getAxis(Joystick.AxisType.kZ);
        }
        
        x *= sensitivity;
        y *= sensitivity;
        rotate *= sensitivity;
        
        // Deadbanding
        if (Math.abs(x) < 0.04) {
            x = 0;
        }
        
        if (Math.abs(y) < 0.04) {
            y = 0;
        }
        
        if (Math.abs(rotate) < 0.04) {
            rotate = 0;
        }
        
        if (fieldcentric) {
        	temp = Math.cos(turned) * x + Math.sin(turned) * y;
        	y = Math.sin(turned) * x + Math.cos(turned) * y;
        	x = temp;
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
        if (Math.abs(gyro.getAngle() - targetAngle) < 1 ||
        		timeOut.get() > 5) {
        	rotating = false;
        	timeOut.stop();
        	timeOut.reset();
        }
        
        if (rotating) {
        	driveTrain.mecanumDrive_Cartesian(x, y, targetAngle, gyro.getAngle());
        } else {
        	driveTrain.mecanumDrive_Cartesian(x, y, rotate, 0.0); // Change once hay gyroscope.
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
        	
        
        // Update button trackers
        changeDriveStyle.update();
        //rotate90Right.update();
        //rotate90Left.update();
        fieldCentric.update();
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

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
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    Talon frontLeft, frontRight, backLeft, backRight;
    Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    RobotDrive driveTrain;
    
    ITG3200_I2C gyro;
    ADXL345_I2C accel;
    
    PositionTracker xDisplacement, yDisplacement, rotationTracker;
    
    Joystick chasis, weapons;
    
    ButtonTracker changeDriveStyle, rotate90Left, rotate90Right, fieldCentric;
    boolean driveStyle, rotating, fieldcentric;
    double targetAngle;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        frontLeft = new Talon(1);
        frontRight = new Talon(4);
        backLeft = new Talon(2);
        backRight = new Talon(3);
        frontLeftEncoder = new Encoder(2, 3);
        backLeftEncoder = new Encoder(4, 5);
        frontRightEncoder = new Encoder(6, 7);
        backRightEncoder = new Encoder(8, 9);
        driveTrain = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
        driveTrain.setInvertedMotor(MotorType.kFrontLeft, true);
        driveTrain.setInvertedMotor(MotorType.kRearLeft, true);
        
        gyro = new ITG3200_I2C(Port.kOnboard);
        accel = new ADXL345_I2C(Port.kOnboard, Accelerometer.Range.k2G); // TODO: Find out exactly what this does.
        
        xDisplacement = new PositionTracker();
        yDisplacement = new PositionTracker();
        rotationTracker = new PositionTracker();
        
        chasis = new Joystick(0);
        weapons = new Joystick(1);

        changeDriveStyle = new ButtonTracker(chasis, 2);
        fieldCentric = new ButtonTracker(chasis, 5);
        driveStyle = false; // False == traditional
        rotate90Left = new ButtonTracker(chasis, 3);
        rotate90Right = new ButtonTracker(chasis, 4);
        rotating = false;
        fieldcentric = true;
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        if (changeDriveStyle.justPressedp()) {
            driveStyle = !driveStyle;
        }
        
        double x, y, rotate, turned, sensitivity, temp;
        turned = rotationTracker.mVelocityIntegral;
        sensitivity = //(chasis.getAxis(Joystick.AxisType.kThrottle) + 1) * 0.25 + 0.5;
                (chasis.getAxis(Joystick.AxisType.kThrottle) * -1 + 1) * 0.25 + 0.5;
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
        	targetAngle = rotationTracker.mVelocityIntegral + 90;
        	rotating = true;
        }
        else if(rotate90Left.justPressedp()){
        	targetAngle = rotationTracker.mVelocityIntegral - 90;
        	rotating = true;
        }
        else {
        	rotating = false;
        }
        if (rotating) {
        	driveTrain.mecanumDrive_Cartesian(x, y, targetAngle, rotationTracker.mVelocityIntegral);
        }
        
        
        
        driveTrain.mecanumDrive_Cartesian(x, y, rotate, 0.0); // Change once hay gyroscope.
        //driveTrain.tankDrive(x, y);
        
        double accelX, accelY;
        accelX = Math.cos(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kX) +
                 Math.sin(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kY);
        accelY = Math.cos(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kX) +
                 Math.sin(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kY);
        xDisplacement.update(accelX);
        yDisplacement.update(accelY);
        rotationTracker.update(gyro.getRate());
        
        changeDriveStyle.update();
        rotate90Right.update();
        rotate90Left.update();
        fieldCentric.update();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}

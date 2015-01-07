/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot2015 extends IterativeRobot {
    Talon frontLeft, frontRight, backLeft, backRight;
    Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    RobotDrive driveTrain;
    
    Gyro gyro;
    ADXL345_I2C accel;
    
    PositionTracker xDisplacement, yDisplacement;
    
    Joystick chasis, weapons;
    
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
        
        gyro = new Gyro(1);
        accel = new ADXL345_I2C(1, ADXL345_I2C.DataFormat_Range.k2G); // TODO: Find out exactly what this does.
        
        xDisplacement = new PositionTracker();
        yDisplacement = new PositionTracker();
        
        chasis = new Joystick(1);
        weapons = new Joystick(2);

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
        double x, y, rotate, turned;
        x = chasis.getAxis(Joystick.AxisType.kTwist);
        y = chasis.getAxis(Joystick.AxisType.kY);
        rotate = chasis.getAxis(Joystick.AxisType.kX);
        turned = gyro.getAngle();
        
        driveTrain.mecanumDrive_Cartesian(x, y, 0.0, 0.0); // Change once hay gyroscope.
        //driveTrain.tankDrive(x, y);
        
        double accelX, accelY;
        accelX = Math.cos(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kX) +
                 Math.sin(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kY);
        accelY = Math.cos(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kX) +
                 Math.sin(turned) * accel.getAcceleration(ADXL345_I2C.Axes.kY);
        xDisplacement.update(accelX);
        yDisplacement.update(accelY);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}

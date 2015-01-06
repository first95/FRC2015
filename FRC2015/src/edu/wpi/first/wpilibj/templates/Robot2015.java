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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot2015 extends IterativeRobot {
    Talon frontLeft, frontRight, backLeft, backRight;
    RobotDrive driveTrain;
    
    Gyro gyro;
    
    Joystick chasis, weapons;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        frontLeft = new Talon(1);
        frontRight = new Talon(2);
        backLeft = new Talon(3);
        backRight = new Talon(4);
        driveTrain = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
        
        gyro = new Gyro(1);
        
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
        x = chasis.getRawAxis(1);
        y = chasis.getRawAxis(3);
        rotate = chasis.getRawAxis(2);
        turned = gyro.getAngle();
        
        driveTrain.mecanumDrive_Cartesian(x, y, rotate, turned);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}

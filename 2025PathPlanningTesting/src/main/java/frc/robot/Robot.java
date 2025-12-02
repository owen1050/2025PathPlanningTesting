// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   Drivebase drivebase = new Drivebase();
   Joystick driverJoystick = new Joystick(0);
   boolean pathInited = false;

  public Robot() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    
  }


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drivebase.swerveDrive.resetOdometry(new Pose2d(1, 1, new Rotation2d()));
  }

  @Override
  public void teleopPeriodic() {
    double speed = -3;
    if(driverJoystick.getRawButton(1)){
      if(!pathInited){
        drivebase.initPath();
        pathInited = true;
      }
      drivebase.pathToTraj();
      drivebase.followPath();
    } else {
      double v = driverJoystick.getRawAxis(1) * driverJoystick.getRawAxis(1) + driverJoystick.getRawAxis(0) * driverJoystick.getRawAxis(0) + driverJoystick.getRawAxis(4) * driverJoystick.getRawAxis(4);
      if(v > 0.1)
        drivebase.driveWithVelocity(driverJoystick.getRawAxis(1) * speed, driverJoystick.getRawAxis(0) * speed, driverJoystick.getRawAxis(4) * -12, true);
      else
        drivebase.driveWithVelocity(0, 0, 0, pathInited);
      pathInited = false;
      drivebase.pathValid = false;
    }
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

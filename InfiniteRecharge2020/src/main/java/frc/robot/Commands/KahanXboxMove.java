/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveBase;

public class KahanXboxMove extends Command {

  double turn;
  double throttle;
  double reverse;

  // Input Buttons
  boolean rotate;
  boolean brake;
  boolean precision;
  boolean gearShiftHigh;
  boolean gearShiftLow;

  /*
   * //Testing Buttons
   * speedConstant1; boolean speedConstant2; boolean speedConstant3;
   */
  // Instance Vars
  double left;
  double right;
  double sensitivity;

  public KahanXboxMove() {
    requires((Subsystem) Robot.drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    ((DriveBase) Robot.drivebase).shiftHighToLow();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*** Read Inputs ***/
    
    //Axes
    /*I don't know what to do here. Uncommnet to see what I mean. Plase tell me if you know what is wrong

    turn = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_AXIS_LEFT_X); 
    throttle = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_AXIS_RIGHT_TRIGGER);
    reverse = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_AXIS_LEFT_TRIGGER);

    //Buttons
    rotate = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_BUTTON_L3);
    brake = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_AXIS_LEFT_TRIGGER);
    precision = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_AXIS_RIGHT_TRIGGER);
    gearShiftLow = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_BUTTON_START);
    gearShiftHigh = Robot.OI.xboxAxis(Robot.OI.xboxDriver, RobotMap.XBOX_BUTTON_BACK);
    */

   //Gear Shift

    if (gearShiftHigh) {
      ((DriveBase) Robot.drivbase).shiftHighToLow();
    } else if (gearShiftLow) {
      ((DriveBase) Robot.drivebase).shiftLowToHigh();
    }

    // Precision 

    if (precision) {
      sensitivity = RobotMap.DRIVE_SENSITIVITY_PRECISION;
    } else {
      sensitivity = RobotMap.DRIVE_SENSITIVITY_DEFAULT;
    }

    //Brake

    if (brake) {
      ((DriveBase) Robot.drivebase).stopMotors(); 
      
      left = 0;
      right = 0;     

      // Comment out the one that is not going to be used
    } else {
      ((DriveBase) Robot.drivebase).setDefaultCommand(new KahanXboxMove()); 
    }

    //Rotation 

    if (rotate) {
      //Joystick is passed Threshold 
      if (Math.abs(turn) > RobotMap.AXIS_THRESHOLD) {
        //Turns to whatever stick is being manipulated since Threeshold requirements have been met
        left = RobotMap.SPIN_SENSITIVITY * turn;
        right = RobotMap.SPIN_SENSITIVITY * (turn * -1);
      } else if (Math.abs(turn) < RobotMap.AXIS_THRESHOLD) {
        //Does not turn since the Threshold requirements have not been met
        left = 0;
        right = 0;
      } else {
        if  (turn > RobotMap.AXIS_THRESHOLD) {
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity * (-1 - turn);
        }
      }
    } else if (turn < (-1 * RobotMap.AXIS_THRESHOLD)) {
        left = (throttle - reverse) * sensitivity * (-1 - turn);
        right = (throttle - reverse) * sensitivity;
      } else {
        left = (throttle - reverse) * sensitivity;
        right = (throttle - reverse) * sensitivity;
      }
      
      // Sends Signal Back to DriveBase
      ((DriveBase) Robot.drivebase).drive(left, right);
    }
  
  @Override
  protected boolean isFinished() {
   return false;
  }

  @Override 
  protected void end() {
    ((DriveBase)Robot.drivebase).stopMotors();
  }

  @Override 
  protected void interrupted() {
    ((DriveBase)Robot.drivebase).stopMotors();
  }

}


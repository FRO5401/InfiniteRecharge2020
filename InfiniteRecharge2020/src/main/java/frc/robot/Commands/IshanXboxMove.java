/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.IshanDrivebase;

public class IshanXboxMove extends Command {
  // Input Axis

  double throttle;
  double turn;
  double reverse;

  // Buttons
  boolean rotate;
  boolean brake;
  boolean precision;
  boolean gearShiftLow;
  boolean gearShiftHigh;

  // Vars
  double sensitivity;
  double left;
  double right;

  public IshanXboxMove() {
    requires(Robot.drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivebase.shiftLowToHigh();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {



    //Axes

    turn = Robot.oi.xboxAxis(Robot.oi.xboxDriver, RobotMap.XBOX_AXIS_LEFT_X);
    reverse = Robot.oi.xboxAxis(Robot.oi.xboxDriver, RobotMap.XBOX_AXIS_LEFT_TRIGGER);
    throttle = Robot.oi.xboxAxis(Robot.oi.xboxDriver, RobotMap.XBOX_AXIS_RIGHT_X);

    // Buttons
    rotate = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_L3);
    brake = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    precision = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
    gearShiftHigh = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_START);
    gearShiftLow = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_BACK);


    //Gear Shifting

    if(gearShiftHigh) {
      Robot.drivebase.shiftHighToLow();

    } else if (gearShiftLow) {
       Robot.drivebase.shiftLowToHigh();
    }


    //Precision

    if(precision) {
      sensitivity = RobotMap.DRIVE_SENSITIVITY_PRECISION;
    }else {
      sensitivity = RobotMap.DRIVE_SENSITIVITY_DEFAULT;
    }
    if (brake) {
      Robot.drivebase.stopMotor(); 
      
      left = 0;
      right = 0;     

      // Comment out the one that is not going to be used
    } else {


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
      Robot.drivebase.drive(left, right);
    }
  }  
  @Override
  protected boolean isFinished() {
   return false;
  }

  @Override 
  protected void end() {
    Robot.drivebase.stopMotor();
  }

  @Override 
  protected void interrupted() {
    Robot.drivebase.stopMotor();
  }

}


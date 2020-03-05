/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

public class XboxMove extends Command {
  
    double turn;
    double throttle;
    double reverse;
  
    boolean rotate; 
    boolean brake;
    boolean precision;
    boolean gearShiftHigh;
    boolean gearShiftLow;
    boolean resetSensors;
    boolean visionInfeed;
  
    double left;
    double right; 
    double sensitivity;
  
    public XboxMove() {
      requires(Robot.drivebase);
    }
  
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
      Robot.drivebase.shiftHighToLow();
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      /*** Read Inputs ***/
        //Axes
      turn = Robot.oi.xboxAxis(Robot.oi.xboxDriver, RobotMap.XBOX_AXIS_LEFT_X);
      throttle = Robot.oi.xboxAxis(Robot.oi.xboxDriver, RobotMap.XBOX_AXIS_RIGHT_TRIGGER);
      reverse = Robot.oi.xboxAxis(Robot.oi.xboxDriver, RobotMap.XBOX_AXIS_LEFT_TRIGGER);
      
        //Buttons
      rotate = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_L3);
      brake = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
      precision = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
      gearShiftHigh = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_START);
      gearShiftLow = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_BACK);
       
      resetSensors = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_A);
      visionInfeed = Robot.oi.xboxButton(Robot.oi.xboxDriver, RobotMap.XBOX_BUTTON_B);

      if(resetSensors){
        Robot.drivebase.resetSensors();
      }    

      if(gearShiftHigh){
        Robot.drivebase.shiftLowToHigh();
      }

      else if(gearShiftLow){
        Robot.drivebase.shiftHighToLow();
      }


      if(visionInfeed == true){
        if(Robot.networktables.getBXValue() != 0 && Robot.networktables.getBallDistance() > 12){
          if(Robot.networktables.getBXValue() > 260 && Robot.networktables.getBXValue() < 380){
            Robot.drivebase.drive(0.35, 0.35);
          }
          else if(Robot.networktables.getBXValue() < 260){
            Robot.drivebase.drive(0.35, 0.45);
          }
          else if(Robot.networktables.getBXValue() > 380){
            Robot.drivebase.drive(0.45, 0.35);
          }
        }
        else{
          Robot.drivebase.stopMotors();
        }
      } 

      else{
        /*** Precision ***/
          //Hold for Precision Speed
        if(precision){
          sensitivity = RobotMap.DRIVE_SENSITIVITY_PRECISION;
        }
          //Release for Regular Speed
        else{
          sensitivity = RobotMap.DRIVE_SENSITIVITY_DEFAULT;
        }
  
        /*** Driving ***/
          //Braking
        if(brake){
          //Robot.drivebase.stopMotors();
          left = 0;
          right = 0;
        }
          //Not Braking
        else{
            //Pirouetting (Turn in place). 
          if(rotate){
              //If the joystick is pushed passed the threshold. 
            if(Math.abs(turn) > RobotMap.AXIS_THRESHOLD){
                //Sets it to spin the desired direction.
              left = RobotMap.SPIN_SENSITIVITY * turn;
              right = RobotMap.SPIN_SENSITIVITY * (turn * -1);
            }
              //If its not past the threshold stop spinning
            else if(Math.abs(turn) < RobotMap.AXIS_THRESHOLD){
              left = 0;
              right = 0;
            }
          }
            //Not pirouetting (Not turning in place).
          else{
              //Turning right
            if(turn > RobotMap.AXIS_THRESHOLD){
                //Makes left slow down by a factor of how far the axis is pushed. 
              left = (throttle - reverse) * sensitivity;
              right = (throttle - reverse) * sensitivity * (1 - turn);
            }
              //Turning left
            else if(turn < (-1 * RobotMap.AXIS_THRESHOLD)){
                //Makes right speed up by a factor of how far the axis is pushed. 
              left = (throttle - reverse) * sensitivity * (1 + turn);
              right = (throttle - reverse) * sensitivity;
            }
              //Driving straight 
            else{
                //No joystick manipulation. 
              left = (throttle - reverse) * sensitivity;
              right = (throttle - reverse) * sensitivity;
            }
          }
        }
          //After speed manipulation, send to drivebase. 
          Robot.drivebase.drive(left, right);
      }
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      return false;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
      Robot.drivebase.stopMotors();
    }
  
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
      Robot.drivebase.stopMotors();
    }
  }
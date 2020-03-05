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
  
  /*** Variables ***/
    //Input Axes
    double turn;
    double throttle;
    double reverse;
  
      //Input Buttons
    boolean rotate; 
    boolean brake;
    boolean precision;
    boolean gearShiftHigh;
    boolean gearShiftLow;
    boolean resetSensors;
  
      /* //Testing Buttons (TODO: Remove for Comp)
    boolean resetSensors;
    boolean speedConstant1;
    boolean speedConstant2;
    boolean speedConstant3;
   */
      //Instance Vars
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

      if(Robot.networktables.getBXValue() != 0){
        if(Robot.networktables.getBXValue() > 260 && Robot.networktables.getBXValue() < 380){
          Robot.drivebase.drive(0.1, 0.1);
        }
        else if(Robot.networktables.getBXValue() < 260){
          Robot.drivebase.drive(0.1, 0.2);
        }
        else if(Robot.networktables.getBXValue() > 380){
          Robot.drivebase.drive(0.2, 0.1);
        }
      }

      else{
        Robot.drivebase.stopMotors();
      }
    } 
    
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
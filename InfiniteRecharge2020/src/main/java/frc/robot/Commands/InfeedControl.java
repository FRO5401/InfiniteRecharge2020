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

public class InfeedControl extends Command {
  boolean infeedIn;
  boolean infeedOut;
  //boolean changeDeployStatus;
  //boolean doneDeploying;

  public InfeedControl() {
    requires(Robot.infeed);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    infeedIn = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
    infeedOut = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    //changeDeployStatus = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_Y);
    //doneDeploying = false; //Needs to be false for initial deploy

    //Infeed Control
    if(infeedIn){
      Robot.infeed.runInfeed();
    }
    
    else if(infeedOut){
      Robot.infeed.reverseInfeed();
    }
    else{
      Robot.infeed.stopInfeed();
    }

    //Deploy control
    /*
    if(changeDeployStatus && doneDeploying == false){
      if(Robot.infeed.getDeployStatus() && doneDeploying == false){
        Robot.infeed.deployInfeed(false);
      }
      else if(!Robot.infeed.getDeployStatus() && doneDeploying == false){
        Robot.infeed.deployInfeed(true);
      }
      doneDeploying = true;
    }

    if(!changeDeployStatus){
      doneDeploying = false;
    }*/
  } 

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.infeed.stopInfeed();
    //Robot.infeed.deployInfeed(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.infeed.stopInfeed();
    //Robot.infeed.deployInfeed(false);
  }
}
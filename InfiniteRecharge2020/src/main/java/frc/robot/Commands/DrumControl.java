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

public class DrumControl extends Command {

  //Buttons
  boolean kick;

  public DrumControl() {
    requires(Robot.drummag);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Update buttons
    kick = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_X);

    //Constantly rotates magazine
    Robot.drummag.rotateMagazine();

    if(kick){
      Robot.drummag.loadBall();
    }
    else{
      Robot.drummag.stopLoading();
    }

  }

  @Override
  protected boolean isFinished() {
    return false; // TODO: Return true when endgame starts to save power
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drummag.stopMagazine();
    Robot.drummag.stopLoading();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drummag.stopMagazine();
    Robot.drummag.stopLoading();
  }
}
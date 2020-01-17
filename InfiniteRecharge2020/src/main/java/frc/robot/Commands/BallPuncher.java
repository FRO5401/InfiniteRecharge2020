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

/*
 * Command controls the Hatch Mechanism.
 * Either OPEN or CLOSE
 */

public class BallPuncher extends Command {
  
  public BallPuncher() {
    requires(Robot.drummag);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
//    Robot.drummag.punchBall();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean ballPunch = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_A);

    if(ballPunch){
      Robot.drummag.punchBall();
    }
    else{
      Robot.drummag.retractPuncher();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
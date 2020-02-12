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
  boolean[] cellLimit = new boolean[5];
  int desiredPosition;
  int position;
  boolean changeMode;
  boolean override;
  boolean homingReset;

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
    position = Robot.drummag.getPosition();
    cellLimit = Robot.drummag.getCellLimits(); //cell limit array
    homingReset = Robot.drummag.getHomingLimit();

    changeMode = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.PLACE_HOLDER);
    override = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.PLACE_HOLDER);

  if(!override){ //If NOT Override button
      desiredPosition = Robot.drummag.findDesiredPosition();

    if(changeMode){
      Robot.drummag.changeMode();
    }

    if(position != desiredPosition){
      Robot.drummag.rotate();
    }

    if(homingReset){
      Robot.drummag.resetPosition();
    }
  }

  else if(override){
    //Override Control
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

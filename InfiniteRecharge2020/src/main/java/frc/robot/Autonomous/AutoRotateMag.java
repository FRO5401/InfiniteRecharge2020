/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoRotateMag extends Command {

  int desiredPosition;
  boolean onTarget;
  int position;

  public AutoRotateMag() {
    onTarget = false;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    position = Robot.drummag.getPosition();
    onTarget = false;
    desiredPosition = Robot.drummag.findDesiredPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(position == desiredPosition){
      onTarget = true;
    }
    else{ 
      if (Robot.drummag.getKickerLimit() == false) { // Stops drummag if kicker is deployed (robot will break if spun while deployed)
        Robot.drummag.stop();
      }    

      else if (Robot.drummag.getKickerLimit() == true) { // Prevents drummag from moving while kicker is deployed    
        if (Robot.drummag.getPosition() != Robot.drummag.findDesiredPosition()) { // Moves until at desired position
          Robot.drummag.rotate();    
          if (Robot.drummag.getGenevaLimit() == false) { // Robot.drummag.finishedRotating will become false once geneve is off limit
            Robot.drummag.switchFinishedRotating();    
          }
        }
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return onTarget;
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

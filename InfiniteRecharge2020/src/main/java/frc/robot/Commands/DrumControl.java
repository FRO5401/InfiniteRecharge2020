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
  boolean kickerLimit;
  boolean genevaOnLimit;

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
    kickerLimit = Robot.drummag.getKickerLimit();
    genevaOnLimit = Robot.drummag.getGenevaLimit();

    changeMode = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.PLACE_HOLDER);
    override = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.PLACE_HOLDER);

  //Switches between shooter and infeed modes when button is pressed
    if(changeMode){
      Robot.drummag.changeMode();
    }

    if(homingReset){ //Resets position when homing limit is tripped
      Robot.drummag.resetPosition();
    }

    if(!override){ //If NOT Override button
        desiredPosition = Robot.drummag.findDesiredPosition(); //Updates desired position
  
      if(kickerLimit){
        Robot.drummag.stop();
      }

      else if(!kickerLimit){ //Prevents drummag from moving while kicker is deployed
        if(position != desiredPosition){ //Moves until at desired position
          Robot.drummag.rotate();
          if(genevaOnLimit == false){
            Robot.drummag.switchFinishedRotating();
          }
        }
        else{
          Robot.drummag.stop();

        }
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
    Robot.drummag.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drummag.stop();
  }
}

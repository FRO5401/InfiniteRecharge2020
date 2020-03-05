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
  
  int desiredPosition;
  int position;

  //Buttons
  boolean changeMode;
  boolean override;
  boolean kick;
  
  //Axis
  double overrideAxis;

  //Limits
  boolean homingLimit;
  boolean kickerLimit;
  boolean genevaLimit;
  boolean[] cellLimit = new boolean[5];

  public DrumControl() {
    requires(Robot.drummag);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // TODO: Add competition reset

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
//    position = Robot.drummag.getPosition();

    // Limits
//    cellLimit = Robot.drummag.getCellLimits(); // cell limit array
//    homingLimit = Robot.drummag.getHomingLimit();
//    kickerLimit = Robot.drummag.getKickerLimit();
//    genevaLimit = Robot.drummag.getGenevaLimit();

    // Buttons
    changeMode = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_START);
    override = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_L3);
    kick = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_X);
    overrideAxis = Robot.oi.xboxAxis(Robot.oi.xboxOperator, RobotMap.XBOX_AXIS_LEFT_X);

    // Puncher, yay! This will only let you punch the ball if the geneva is on limit
    if (kick) {
      Robot.drummag.punchBall(true);
    } else {
      Robot.drummag.punchBall(false);
    }

    // Switches between shooter and infeed modes when button is pressed
/*    if (changeMode) {
      Robot.drummag.changeMode();
    } */

/*    if (homingLimit) { // Resets position when homing limit is tripped
      Robot.drummag.resetPosition();
    } */

//    if(kickerLimit == true){
/*      if (!override) { // If NOT Override button
        desiredPosition = Robot.drummag.findDesiredPosition(); // Updates desired position

        if (position != desiredPosition) { // Moves until at desired position
          Robot.drummag.rotate();
          if (genevaLimit == false) { // Robot.drummag.finishedRotating will become false once geneve is off limit
            Robot.drummag.switchFinishedRotating();
          } 
        }
        else { // When position reaches desired position
          Robot.drummag.stop();
        }
      } */
      if (override) {
        if(overrideAxis > RobotMap.AXIS_THRESHOLD){
          Robot.drummag.rotate();
/*          if(genevaLimit == false){
            Robot.drummag.switchFinishedRotating();
          } 
        } */
      }
    }
    else if(kickerLimit == false){
      Robot.drummag.stop();
    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; // TODO: Return true when endgame starts to save power
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
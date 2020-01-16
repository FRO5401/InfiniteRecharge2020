/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.RobotMap;
//import frc.robot.Subsystems.DrumMag;

/*
 * Command controls PID setpoints.
 * "Supposed" to run once Override is done running. 
 */ 

public class DrumMagPID extends Command {
 
    //Constants
  int slot1, slot2, slot3, slot4, slot5;

    //Limit switches
  boolean ballLimit1, ballLimit2, ballLimit3, ballLimit4, ballLimit5;

  public DrumMagPID() {
    requires(Robot.drummag);

      //Setpoints
      slot1 = 0;
      slot2 = 72;
      slot3 = 144;
      slot4 = 216;
      slot5 = 288;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Infeed button
    boolean rotateToInfeed = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);

    //Shooter button
    boolean rotateToShooter = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_Y);

      //Read Limit Switches
    ballLimit1 = Robot.drummag.slot1Status();
    ballLimit2 = Robot.drummag.slot2Status();
    ballLimit3 = Robot.drummag.slot3Status();
    ballLimit4 = Robot.drummag.slot4Status();
    ballLimit5 = Robot.drummag.slot5Status();


    //***Logic for Drum Mag rotation based on conditions***//

    //Bring Slot 1 to face Infeed
    if(rotateToInfeed = true) {
        Robot.drummag.setPoint(slot1);
    }

    //Logic for determining when to turn to the next slot when infeeding
    if((ballLimit1 = true) && Robot.drummag.getCurrentSlot() == 1){ //Turn to slot 2 when ball is in slot 1     
        Robot.drummag.setPoint(slot2);
    }
    if((ballLimit2 = true) && Robot.drummag.getCurrentSlot() == 2){ //Turn to slot 2 when ball is in slot 1     
        Robot.drummag.setPoint(slot3);
    }
    if((ballLimit3 = true) && Robot.drummag.getCurrentSlot() == 3){ //Turn to slot 2 when ball is in slot 1     
        Robot.drummag.setPoint(slot4);
    }  
    if((ballLimit4 = true) && Robot.drummag.getCurrentSlot() == 4){ //Turn to slot 2 when ball is in slot 1     
        Robot.drummag.setPoint(slot5);
    }
    if((ballLimit5 = true) && (ballLimit1 = !true) && Robot.drummag.getCurrentSlot() == 5) {
        Robot.drummag.setPoint(slot1);
    }


    //Bring Slot 1 to face Shooter
    else if(rotateToShooter = true) {
        Robot.drummag.setPoint(slot4 - 36);
    }    

    //Logic for determining when to turn to the next slot when shooting
    if((ballLimit1 = false) && Robot.drummag.getCurrentSlot() == 3.5){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(slot2);
    }
    if((ballLimit2 = false) && Robot.drummag.getCurrentSlot() == 4.5){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(slot3);
    }
    if((ballLimit3 = false) && Robot.drummag.getCurrentSlot() == 0.5){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(slot4);
    }  
    if((ballLimit4 = false) && Robot.drummag.getCurrentSlot() == 1.5){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(slot5);
    }
    if((ballLimit5 = false) && (ballLimit1 = true) && Robot.drummag.getCurrentSlot() == 2.5) {
        Robot.drummag.setPoint(slot1);
    }    
  }

    

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drummag.onTarget();
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
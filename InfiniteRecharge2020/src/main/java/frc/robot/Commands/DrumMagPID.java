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
//import frc.robot.RobotMap;
//import frc.robot.Subsystems.DrumMag;

/*
 * Command controls PID setpoints.
 * "Supposed" to run once Override is done running. 
 */ 

public class DrumMagPID extends Command {
 
    //Constants
  int slot1, slot2, slot3, slot4, slot5;
  int shootSlot1, shootSlot2, shootSlot3, shootSlot4, shootSlot5;

    //Limit switches
  boolean ballLimit1, ballLimit2, ballLimit3, ballLimit4, ballLimit5;
  boolean facingShooter;

  public DrumMagPID() {
    requires(Robot.drummag);

      //Pickup Setpoints (in degrees)
      slot1 = 0;
      slot2 = 72;
      slot3 = 144;
      slot4 = 216;
      slot5 = 288;

      //Shoot Setpoints (in degrees)
      shootSlot1 = 180;
      shootSlot2 = 252;
      shootSlot3 = 324;
      shootSlot4 = 36;
      shootSlot5 = 108;

      facingShooter = false;


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drummag.setPoint(0);
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
    if(rotateToInfeed) {
        Robot.drummag.setPoint(slot1); //Set back to 0 degrees
        facingShooter = false;
        
    }

    //Logic for determining when to turn to the next slot when infeeding
    //Maybe change to else if
    if((ballLimit1 = true) && Robot.drummag.getCurrentSlot() == 1){ //Turn to slot 2 when ball is in slot 1     
        Robot.drummag.setPoint(slot2);
    }
    if((ballLimit2 = true) && Robot.drummag.getCurrentSlot() == 2){ //Turn to slot 3 when ball is in slot 2     
        Robot.drummag.setPoint(slot3);
    }
    if((ballLimit3 = true) && Robot.drummag.getCurrentSlot() == 3){ //Turn to slot 4 when ball is in slot 3     
        Robot.drummag.setPoint(slot4);
    }  
    if((ballLimit4 = true) && Robot.drummag.getCurrentSlot() == 4){ //Turn to slot 5 when ball is in slot 4     
        Robot.drummag.setPoint(slot5);
    }
    if((ballLimit5 = true) && (ballLimit1 = !true) && Robot.drummag.getCurrentSlot() == 5) {
        Robot.drummag.setPoint(slot1);
    }


    //Bring Slot 1 to face Shooter
    if(rotateToShooter) {
        Robot.drummag.setPoint(shootSlot1); //Set to 180 degrees
        facingShooter = true;
    }    

    //Logic for determining when to turn to the next slot when shooting
    if((ballLimit1 = false) && (facingShooter = true)){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(shootSlot2);
    }
    if((ballLimit2 = false) &&  (facingShooter = true)){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(shootSlot3);
    }
    if((ballLimit3 = false) && (facingShooter = true)){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(shootSlot4);
    }  
    if((ballLimit4 = false) && (facingShooter = true)){ //Turn to slot 2 when ball leaves slot 1     
        Robot.drummag.setPoint(shootSlot5);
    }
    if((ballLimit5 = false) && (ballLimit1 = true) && (facingShooter = true)) {
        Robot.drummag.setPoint(shootSlot1);
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
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * An example command. You can replace me with your own command.
 */
public class TurretControl extends Command {

  // Creates the limits, turn functions, and buttons needed for the program.
  boolean limitRight, limitLeft;
  boolean resetButton;
  boolean overrideToggle;
  boolean controlVision;
  double turretRotate;
  boolean resetPosition;
  double xVision;
  int dPad;

  // Constructor that finds the subsystem
  public TurretControl() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.turret.turretSetTalonNeutralMode(NeutralMode.Brake);
    //Robot.turret.resetTurretAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    overrideToggle = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_R3);
    controlVision = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_A);
    resetPosition = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_B);

    if (resetPosition){
      Robot.turret.goToAngle(0.0);
    }

    // Checks to see if the ready button was pushed
    else if (overrideToggle) {
      //Disables vision if on
      System.out.println("override");
      Robot.turret.disableVision();

      //Allows turret to move if Joystick axis is above treshold
      if (turretRotate > RobotMap.AXIS_THRESHOLD || turretRotate < (-1 * RobotMap.AXIS_THRESHOLD)) {
        Robot.turret.moveTurret(turretRotate);
      } 
      else {
        Robot.turret.stopRotation();
      }
    }

    //If the turret isn't recieving commands, don't move!!
    else{
      System.out.println("no override");
      Robot.turret.stopRotation();
    }

  } // All elses stop rotation as conditions are not met.

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.turret.stopRotation();
    Robot.turret.resetTurretAngle();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.turret.stopRotation();
    Robot.turret.resetTurretAngle();
  }
}
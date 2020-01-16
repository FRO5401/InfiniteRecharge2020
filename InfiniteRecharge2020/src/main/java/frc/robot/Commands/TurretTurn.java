/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
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
 * An example command.  You can replace me with your own command.
 */
public class TurretTurn extends Command {

  boolean limitRight;
  boolean limitLeft;
  double turnRight;
  double turnLeft;
  boolean resetButton;
  boolean readyButton;
  double turretLeftRight;

  public TurretTurn() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.turret.turretSetTalonNeutralMode(NeutralMode.Brake);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    resetButton = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_B); 
    readyButton = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_Y);

    turretLeftRight = Robot.oi.xboxAxis(Robot.oi.xboxOperator, RobotMap.XBOX_AXIS_RIGHT_X);
    if (resetButton){
      Robot.turret.resetTurretAngle();
    }

    if(readyButton){
      if (!limitLeft && !limitRight){
        if (turretLeftRight > RobotMap.AXIS_THRESHOLD) {
          Robot.turret.rotateTurretRight();
        }
        else if (turretLeftRight < (-1 * RobotMap.AXIS_THRESHOLD)) {
          Robot.turret.rotateTurretLeft();
        }
        else if (turretLeftRight == RobotMap.AXIS_THRESHOLD) {
          Robot.turret.stopRotation();
        }
      }
      else {
        Robot.turret.stopRotation();
      }
    }
    else{
      Robot.turret.stopRotation();
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
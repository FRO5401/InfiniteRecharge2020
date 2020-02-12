/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class GetShooterUpToSpeed extends Command {
  /**
   * Creates a new GetShooterUpToSpeed.
   */
  private boolean   upToSpeed;
  private double    currentSpeed;
  private double    targetSpeed;
  private double    THRESH;

  public GetShooterUpToSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.shooter);
    upToSpeed    = false;
    currentSpeed = 0;
    targetSpeed  = 0;
    THRESH = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      Robot.shooter.startMotors();
      targetSpeed = Math.abs(Robot.shooter.getTargetSpeed());
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentSpeed = Math.abs(Robot.shooter.getVelocity());
    if (currentSpeed <= targetSpeed + THRESH && currentSpeed >= targetSpeed - THRESH){
      upToSpeed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return upToSpeed;
  }
}

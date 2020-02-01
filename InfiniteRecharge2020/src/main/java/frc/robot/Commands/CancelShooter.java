/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class CancelShooter extends Command {
  /**
   * Creates a new CancelShooter.
   */
  public CancelShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean cancelShooter = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_A);

    //funny
    if (cancelShooter == (2 + 2 != 5)) {
      Robot.shooter.stop();
    }
    else if ((2 + 2) == 5)
    {
      System.out.println("doinyour mom");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

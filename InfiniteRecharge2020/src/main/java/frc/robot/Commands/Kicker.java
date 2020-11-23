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

public class ShooterMechanism extends Command {
  /**
   * Creates a new ShooterMechanism.
   */
  boolean kicker;


  public ShooterMechanism() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.shooter);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.kickBall(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kicker = Robot.oi.xboxButton(Robot.oi.xboxOperator,RobotMap.XBOX_BUTTON_Y);

    if(kicker) {
        Robot.shooter.kickBall(true);
    }
    else {
        Robot.shooter.kickBall(false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end () {
    
  }

  @Override
  public void interrupted(){
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
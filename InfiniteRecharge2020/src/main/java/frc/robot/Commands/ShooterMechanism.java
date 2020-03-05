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
  boolean readyShooter;
  boolean cancelShooter;

  //For PID testing
  public int dPad;


  public ShooterMechanism() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.shooter);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.startMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.shooter.reportValues();
    readyShooter  = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_Y);
    cancelShooter = Robot.oi.xboxButton(Robot.oi.xboxOperator,RobotMap.XBOX_BUTTON_B);

    //For Pid testing
    dPad = Robot.oi.xboxDPad(Robot.oi.xboxOperator);

    //For PID testing
    if(dPad == 180){
      Robot.shooter.getPIDInput();
    }

    if (cancelShooter) {
      Robot.shooter.stop();
    }

    if(readyShooter) {
      Robot.shooter.runMotors();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end () {
    Robot.shooter.stop();
  }

  @Override
  public void interrupted(){
    Robot.shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
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
import frc.robot.Subsystems.Shooter;

public class ShooterMechanism extends Command {
  /**
   * Creates a new ShooterMechanism.
   */
  boolean kickerButton;
  boolean reverseKicker;
  double shooterButton;
  boolean shooterRunning = false;

  //For PID testing
  public int dPad;


  public ShooterMechanism() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.shooter);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Maybe put a stop here later idk
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //NOTE: We have to put code for expelling the balls from the shooter in case of a jam

    Robot.shooter.reportValues();
    kickerButton = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_Y);
    shooterButton = Robot.oi.xboxAxis(Robot.oi.xboxOperator, RobotMap.XBOX_AXIS_RIGHT_TRIGGER);
    reverseKicker = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_X);
    //For Pid testing
    dPad = Robot.oi.xboxDPad(Robot.oi.xboxOperator);

    //For PID testing
    if(dPad == 180){
      Robot.shooter.getPIDInput();
    }


    if(shooterButton > RobotMap.AXIS_THRESHOLD) {
      Robot.shooter.runMotors();
    }
    else {
      Robot.shooter.stopMotors();
    }

    if(kickerButton) {
      Robot.serializer.runKicker("IN");
    }
    else if (reverseKicker) {
      Robot.serializer.runKicker("OUT");
    }
    else {
      Robot.serializer.runKicker("STOP");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end () {
    Robot.shooter.stopMotors();
    Robot.serializer.runKicker("STOP");
  }

  @Override
  public void interrupted(){
    Robot.shooter.stopMotors();
    Robot.serializer.runKicker("STOP");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
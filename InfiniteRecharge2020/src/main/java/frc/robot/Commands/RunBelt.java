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

public class RunBelt extends Command {
  /**
   * Creates a new ShooterMechanism.
   */
  boolean serializer;


  public RunBelt() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.serializer);  
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

    serializer = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_B);

    if(serializer) {
        Robot.serializer.runSerializer("IN");
    }
    else {
      Robot.serializer.runSerializer("STOP");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end () {
    Robot.serializer.runSerializer("STOP");
  }

  @Override
  public void interrupted(){
    Robot.serializer.runSerializer("STOP");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
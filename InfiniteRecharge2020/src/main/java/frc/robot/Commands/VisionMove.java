package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/*
 * Command controls the Hatch Mechanism.
 * Either OPEN or CLOSE
 */

public class VisionMove extends Command {

  public double xVision;

  public VisionMove() {
    requires(Robot.networktables);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //xVision = Robot.networktables.getXValue();
    // put automatic code here to be used during teleop, maybe add a button to
    // activate it (not sure if there is one yet)
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
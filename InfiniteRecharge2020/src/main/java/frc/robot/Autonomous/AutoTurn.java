/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class AutoTurn extends Command {

  private double desiredAngle;
	private double autoTurnSpeed;
	public boolean isFinished;
	private double angleTraveled;
  public AutoTurn(double angleInput, double speedInput) {
    
    desiredAngle = angleInput;
    autoTurnSpeed = speedInput;
    isFinished = false;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivebase.resetSensors();
    isFinished = false;
    angleTraveled = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    angleTraveled = Robot.drivebase.getGyroAngle();
    if ((angleTraveled) <= (desiredAngle - RobotMap.ANGLE_THRESHOLD) && desiredAngle > 0){
			Robot.drivebase.drive(autoTurnSpeed, (-1 * autoTurnSpeed));
			isFinished = false;
		}
		else if(angleTraveled >= (desiredAngle + RobotMap.ANGLE_THRESHOLD) && desiredAngle < 0){
      Robot.drivebase.drive((-1 * autoTurnSpeed), autoTurnSpeed);
      isFinished = false;
		}
		else{
			Robot.drivebase.stopMotors();
			isFinished = true;
		}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.print("Done turning.");
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivebase.stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drivebase.stopMotors();
  }
}
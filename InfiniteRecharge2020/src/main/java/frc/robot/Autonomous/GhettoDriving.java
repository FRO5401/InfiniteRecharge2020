/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class GhettoDriving extends Command {

	private double desiredDistance;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;
	private double heading;

	public GhettoDriving() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);
		doneTraveling = true;
		distanceTraveled = 0;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		doneTraveling = false;
		distanceTraveled = 0;


	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drivebase.drive(0.5, 0.5);
		try {
			Thread.sleep(1000);
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
		Robot.drivebase.stopMotors();
		Robot.drivebase.drive(-0.5, 0.5);
		try {
			Thread.sleep(250);
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
		Robot.drivebase.stopMotors();
		Robot.shooter.runMotors();
		try {
			Thread.sleep(100);
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
		Robot.serializer.runKicker("IN");
		try {
			Thread.sleep(100);
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
		Robot.serializer.runSerializer("IN");
		try {
			Thread.sleep(4000);
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
		Robot.shooter.stopMotors();
		Robot.serializer.runSerializer("STOP");
		Robot.serializer.runKicker("STOP");
		doneTraveling = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return doneTraveling;
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
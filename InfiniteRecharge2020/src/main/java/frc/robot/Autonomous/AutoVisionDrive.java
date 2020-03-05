/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoVisionDrive extends Command {

    //private int ballCount;
	private double angle;
	private double desiredDistance;
	private double autoDriveSpeed;
	private double distanceTraveled;
	private boolean doneTraveling;

	public AutoVisionDrive(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
        distanceTraveled = 0;
        //initialCount = 0;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		Robot.drivebase.resetSensors();
		Robot.drivebase.setDPPHighGear();
		Robot.drivebase.setDPPLowGear();

		doneTraveling = false;
		distanceTraveled = 0;

		try {
			desiredDistance = (Robot.networktables.getBallDistance());
		}
		catch (NullPointerException e) {
			desiredDistance = 0;
		} 

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		angle = Robot.drivebase.navxGyro.getAngle();
		distanceTraveled = Robot.drivebase.getEncoderDistance(2) * RobotMap.LOW_GEAR_RIGHT_DPP;
        //Robot.infeed.runMotors();

		if ((distanceTraveled) < (desiredDistance - 5)) {
			Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
			doneTraveling = false;
		} 
		else {
            //if(Robot.drummag.getBallCount() > initialCount){
			    Robot.drivebase.stopMotors();
                doneTraveling = true;
            //}
		}
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
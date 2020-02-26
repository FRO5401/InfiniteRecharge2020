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

public class AutoBallInfeed extends Command {

    private double desiredDistance;
    private double desiredAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;
    private double heading;
    
    private boolean isCentered;

	public AutoBallInfeed(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

        desiredDistance = Robot.networktables.getBallDistance();
        desiredAngle = Robot.networktables.getBXValue();
        isCentered = Robot.drivebase.checkCentered();
		// Distance is 127 inches not considering robot size

		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
		distanceTraveled = 0;
		// heading = Robot.drivebase.getGyroAngle();

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		Robot.drivebase.resetSensors();
		Robot.drivebase.setDPPHighGear();
		Robot.drivebase.setDPPLowGear();
		// heading = Robot.drivebase.getGyroAngle();
		doneTraveling = false;
		distanceTraveled = 0;
		// navXPitchInit = Robot.drivebase.getGyroPitch();

		// System.out.println("AutoDriveInitializing");
		// System.out.println("Angle when starting DriveShift:" +
		// Robot.drivebase.getGyroAngle());
		SmartDashboard.putNumber("heading", heading);
		// Robot.drivebase.shiftGearHighToLow();

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//distanceTraveled = Robot.drivebase.getEncoderDistance(2) * RobotMap.LOW_GEAR_RIGHT_DPP;
		if(isCentered == true) {
                if ((distanceTraveled) <= (desiredDistance) && desiredDistance >= 0) {
			    Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed);
			    doneTraveling = false;
		    } else if (distanceTraveled >= (desiredDistance) && desiredDistance < 0) {
			    Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed);
		    } else {
			    Robot.drivebase.stopMotors();
			    doneTraveling = true;
            }
        }
        else {
            Robot.drivebase.autoTurn(desiredAngle, autoDriveSpeed);
        }

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		System.out.print("Should be finished");
		return doneTraveling;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivebase.stopMotors();
		// System.out.println("Angle when EXITING DriveShift:" +
		// Robot.drivebase.getGyroAngle());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.drivebase.stopMotors();
	}

}
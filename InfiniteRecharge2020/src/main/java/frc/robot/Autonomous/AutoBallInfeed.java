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
	private double radius;

	private final double turnThresh;
    
    private boolean isCentered;

	public AutoBallInfeed(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

        desiredDistance = Robot.networktables.getBallDistance();
        desiredAngle = Robot.networktables.getBXValue();
		// Distance is 127 inches not considering robot size

		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
		distanceTraveled = 0;

		turnThresh = 3;
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
		radius = Robot.networktables.getBallRadius();
		isCentered = Robot.drivebase.checkCentered();
		
		if(radius == 0){ //If no ball is recognized, scan area
			Robot.drivebase.autoTurn(90, 0.8);
			Robot.drivebase.autoTurn(-180, 0.8);
		}
		else if(Robot.networktables.radius > 0){ //If ball is recognized drive towards it and infeed
			if(isCentered == true) { //Once recognized ball is straight ahead, drive towards it based off of received distance
				//Robot.infeed.startMotors();
            	if ((distanceTraveled) <= (desiredDistance) && desiredDistance >= 0) {
			    	Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed);
			    	doneTraveling = false;
				} 
				else if (distanceTraveled >= (desiredDistance) && desiredDistance < 0) {
			    	Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed);
				} 
				else {
			    	Robot.drivebase.stopMotors();
			    	doneTraveling = true;
            	}
        	}
        	else { //Turn until the ball that is recognized is straight ahead
				if(Robot.networktables.getBXValue() > (turnThresh * -1)){
					Robot.drivebase.autoTurn(5, autoDriveSpeed);
				}
            	else if(Robot.networktables.getBXValue() < turnThresh){
					Robot.drivebase.autoTurn(-5, autoDriveSpeed);
				}
        	}
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
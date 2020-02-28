/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
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
    
    private double startTime;
    private double currentTime;

	private final double turnThresh;
    
    private boolean isCentered;

	public AutoBallInfeed(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

        //desiredDistance = Robot.networktables.getBallDistance();
		// Distance is 127 inches not considering robot size

		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
		distanceTraveled = 0;

		turnThresh = 5;
		// heading = Robot.drivebase.getGyroAngle();

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        startTime = Timer.getMatchTime();

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
        System.out.println("yes");
        currentTime = Timer.getMatchTime();
        SmartDashboard.putNumber("Time elapsed", startTime - currentTime);

		radius = Robot.networktables.getBallRadius();
		isCentered = Robot.drivebase.checkCentered();
        desiredAngle = Robot.networktables.getBXValue();
        
        if((startTime - currentTime >= 3) & (radius == 0)){//If no ball has been found after 3 seconds, go back to original angle and stop
            
            if(Robot.drivebase.navxGyro.getAngle() < turnThresh && Robot.drivebase.navxGyro.getAngle() > -turnThresh){
                doneTraveling = true;
            }
            else if(Robot.drivebase.navxGyro.getAngle() > turnThresh){
                Robot.drivebase.autoTurn(-10, 0.1);
            }
            else if(Robot.drivebase.navxGyro.getAngle() < -turnThresh){
                Robot.drivebase.autoTurn(10, 0.1);
            }
            
        }
        if(radius == 0){ //If no ball is recognized, scan area
            if(startTime - currentTime < 3 & radius == 0){
                Robot.drivebase.autoTurn(360, 0.5);
            }
		}
		else if(Robot.networktables.radius > 0){ //If ball is recognized drive towards it and infeed
		    if(isCentered == true) { //Once recognized ball is straight ahead, drive towards it based off of received distance
			    //Robot.infeed.startMotors();
                if ((distanceTraveled) <= (desiredDistance) && desiredDistance >= 0) {
		    	    Robot.drivebase.drive(autoDriveSpeed, autoDriveSpeed);
		    	    doneTraveling = false;
			    } 
			    else if (distanceTraveled >= (desiredDistance) && desiredDistance < 0) {
		    	    Robot.drivebase.drive(autoDriveSpeed, autoDriveSpeed);
			    } 
			    else {
		    	    Robot.drivebase.stopMotors();
		    	    doneTraveling = true;
                }
            }
    	    else { //Turn until the ball that is recognized is straight ahead
			    if(desiredAngle < (turnThresh * -1)){
				    Robot.drivebase.autoTurn(-5, autoDriveSpeed);
			    }
        	    else if(desiredAngle > turnThresh){
				    Robot.drivebase.autoTurn(5, autoDriveSpeed);
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
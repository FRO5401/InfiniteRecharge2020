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

public class AutoVisionCenter extends Command {

    //private double currentAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double portLocation;
    
    private double startTime;
    private double currentTime;
    
	private boolean isCentered;

	public AutoVisionCenter(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);s
		autoDriveSpeed = SpeedInput;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		startTime = Timer.getMatchTime();
		
		Robot.drivebase.resetSensors();
		Robot.drivebase.setDPPHighGear();
		Robot.drivebase.setDPPLowGear();

		doneTraveling = false;
		isCentered = false;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		currentTime = Timer.getMatchTime();
		double timeElapsed = startTime - currentTime;
        SmartDashboard.putNumber("Time elapsed", timeElapsed);

		if(isCentered == false){
			isCentered = Robot.networktables.checkCenteredPort();

		}
		if(portLocation > 0){ //If power port is recognized, turn to center drivebase
		    if(isCentered == true) { 
				Robot.drivebase.stopMotors();
				doneTraveling = true;
            }
			else { //Turn until the port that is recognized is straight ahead
				if(portLocation > Robot.networktables.leftBound){//turn right
					Robot.drivebase.drive(autoDriveSpeed, -1 * autoDriveSpeed);
					doneTraveling = false;
				}
				else if(portLocation < Robot.networktables.rightBound){//turn left
					Robot.drivebase.drive(-1 * autoDriveSpeed, autoDriveSpeed);
					doneTraveling = false;
				}
            }
        }
		else if(portLocation == 0){ //If no port is recognized, scan area
			isCentered = false;
			if(timeElapsed >= 3){//If no port has been found after 3 seconds, go back to original angle and stop
				if(Robot.drivebase.navxGyro.getAngle() > (Robot.drivebase.navxGyro.getAngle() % 366)){
					Robot.drivebase.drive(-1 * autoDriveSpeed, autoDriveSpeed);
				}
				else if(Robot.drivebase.navxGyro.getAngle() < (Robot.drivebase.navxGyro.getAngle() % 366)){
					Robot.drivebase.drive(autoDriveSpeed, -1 * autoDriveSpeed);
				}
				else{
					Robot.drivebase.stopMotors();
					doneTraveling = true;
					Robot.drivebase.navxGyro.reset();
				}	
			}
			else if((timeElapsed) < 3){
				Robot.drivebase.drive(-1 * 0.3, (0.3));
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
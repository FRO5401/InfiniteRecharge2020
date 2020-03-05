/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoBallInfeed extends Command {

    private double autoDriveSpeed;
    private double ballLocation;
    private boolean doneTraveling;
    
    private double startTime;
    private double currentTime;

	public AutoBallInfeed(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
        // requires(Robot.drivebase);
        doneTraveling = false;
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

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
      currentTime = Timer.getMatchTime();
		  double timeElapsed = startTime - currentTime;
      SmartDashboard.putNumber("Time elapsed", timeElapsed);
      ballLocation = Robot.networktables.getBXValue();
        
      //Robot.infeed.runInfeed();
      if(ballLocation == 0 && (timeElapsed) < 5){
          Robot.drivebase.drive(0.35, -1 * 0.35);
      }

		  else if(ballLocation != 0 && Robot.networktables.getBallDistance() > 12){
          if(ballLocation > 260 && ballLocation < 380){
              Robot.drivebase.drive(autoDriveSpeed, autoDriveSpeed);
              doneTraveling = false;
          }
          else if(ballLocation < 260){
              Robot.drivebase.drive(autoDriveSpeed, 1.5 * autoDriveSpeed);
              doneTraveling = false;
          }
          else if(ballLocation > 380){
              Robot.drivebase.drive(1.5 * autoDriveSpeed, autoDriveSpeed);
              doneTraveling = false;
          }
      }
      else if (ballLocation != 0){
          Robot.drivebase.stopMotors();
          doneTraveling = true;
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
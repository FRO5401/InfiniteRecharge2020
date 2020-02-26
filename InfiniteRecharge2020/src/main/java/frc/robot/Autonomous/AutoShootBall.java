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

public class AutoShootBall extends Command {

    private int ballCount;
	private boolean magEmpty;
	private double heading;

	public AutoShootBall() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

        //ballCount = Robot.drummag.ballCount();
        magEmpty = true;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

        //Robot.drummag.magMode = 1;
		Robot.drivebase.resetSensors();
		Robot.drivebase.setDPPHighGear();
		Robot.drivebase.setDPPLowGear();

		magEmpty = false;
		
		SmartDashboard.putNumber("heading", heading);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
    /*
        if(ballCount == 0){
            magEmpty = true;
        }
        else if(ballCount > 0){
            if(position = desiredPosition){
                //Robot.drummag.punchBall(true);
                if(currentBall == false){
                    //Robot.drummag.punchBall(false);
                }
                else{
                    //Robot.drummag.punchBall(false);
                }
                if(Robot.drummag.kickerLimit == true){
                    //Robot.drummag.findDesiredPosition();
                    //Robot.drummag.rotate();
                else{

                }
                magEmpty = false;
            }
        }
    */
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return magEmpty;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        Robot.drivebase.stopMotors();
        Robot.shooter.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
        Robot.drivebase.stopMotors();
        Robot.shooter.stopMotors();
	}

}
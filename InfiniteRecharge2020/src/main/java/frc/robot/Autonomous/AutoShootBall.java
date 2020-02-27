/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoShootBall extends Command {

    private int ballCount;
    private int position;
    private int desiredPosition;
	private boolean magEmpty;
	private double heading;

	public AutoShootBall() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

        ballCount = Robot.drummag.ballCount();
        magEmpty = true;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

        Robot.turret.enableVision();
        Robot.shooter.runMotors();

        Robot.drummag.magMode = 1;
		Robot.drivebase.resetSensors();
		Robot.drivebase.setDPPHighGear();
		Robot.drivebase.setDPPLowGear();

		magEmpty = false;
		
		SmartDashboard.putNumber("heading", heading);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        desiredPosition = Robot.drummag.findDesiredPosition();
        position = Robot.drummag.getPosition(); 

        if(ballCount == 0){
            magEmpty = true;
        }
        else if(ballCount > 0){
            if(position == desiredPosition){
                Robot.drummag.punchBall(true);
                if(position != desiredPosition && Robot.drummag.getKickerLimit() == false){
                    Robot.drummag.punchBall(false);
                }
            }
            else{
                if(Robot.drummag.getKickerLimit() == true){
                    Robot.drummag.rotate();
                    if (Robot.drummag.getGenevaLimit() == false) { // Robot.drummag.finishedRotating will become false once geneve is off limit
                        Robot.drummag.switchFinishedRotating();
                    }
                }
            }
            magEmpty = false;   
        }
	}
        //if(ballCount == 0){
            //magEmpty = true;
        //}
        //else if(ballCount > 0){
            /*magEmpty = false;
            if (Robot.drummag.getKickerLimit() == false) { // Stops drummag if kicker is deployed (robot will break if spun while deployed)
            Robot.drummag.stop();
            }    

            else if (Robot.drummag.getKickerLimit() == true) { // Prevents drummag from moving while kicker is deployed    
                if (Robot.drummag.getPosition() != Robot.drummag.findDesiredPosition()) { // Moves until at desired position
                    Robot.drummag.rotate();    
                    if (Robot.drummag.getGenevaLimit() == false) { // Robot.drummag.finishedRotating will become false once geneve is off limit
                    Robot.drummag.switchFinishedRotating();    
                    }
                } 
            }
            */
        
        
        

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
        Robot.turret.disableVision();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
        Robot.drivebase.stopMotors();
        Robot.shooter.stopMotors();
	}

}
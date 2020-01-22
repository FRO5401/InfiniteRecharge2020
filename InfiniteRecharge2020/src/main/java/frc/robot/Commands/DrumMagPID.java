/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.RobotMap;
//import frc.robot.Subsystems.DrumMag;

/*
 * Command controls PID setpoints.
 * "Supposed" to run once Override is done running. 
 */

public class DrumMagPID extends Command {

    public DrumMagPID() {
        requires(Robot.drummag);

        //double target = Robot.drummag.getMagAngle() + (Robot.drummag.getMagAngle() % 36);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
        // Infeed button
        boolean rotateToInfeed = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
        // Shooter button
        boolean rotateToShooter = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_Y);
        // Ball Puncher button
        boolean ballPunch = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_A);
        // Stop shooting button
        boolean cancelShooter = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_B);  


        //***Logic for Drum Mag rotation based on conditions and 'DrumMag' methods***//

        // Bring Slot 1 to face Infeed through button
        if (rotateToInfeed && (Robot.drummag.getMode() == "Shooter")) { //If not already in infeed mode, switch to infeed
            Robot.drummag.rotateToInfeed();
        }
        if (Robot.drummag.getMode() == "Infeed") { //If in infeed mode, run through the method in 'DM' for filling
            Robot.drummag.infeedBalls();
        }

        // Bring Slot 1 to face Shooter through button
        if (rotateToShooter && (Robot.drummag.getMode() == "Infeed")) { //If not already in shooter mode, switch to shooter
            Robot.drummag.rotateToShooter();
        }
        if (Robot.drummag.getMode() == "Shooter") { //if in shooter mode, run through the method in 'DM' for emptying
            Robot.drummag.shootBalls();
        }

        //If shooting override is pressed, turn to face infeed and stop shooting process. 
            //Meant to ensure DrumMag and ballPuncher won't keep going when unwanted 
        if (cancelShooter && (Robot.drummag.getMode() == "Shooter")) { //Must be in shooter mode to make sure drummag won't rotate back when already in infeed mode
            Robot.drummag.rotateToInfeed();
        }

        //If shooter is ready and button pressed, ball will be punched
        if(ballPunch){
            Robot.drummag.punchBall();
        }
        else {
            Robot.drummag.retractPuncher();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.drummag.onTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
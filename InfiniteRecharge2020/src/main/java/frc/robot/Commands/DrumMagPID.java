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
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.drummag.setPoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        // Read Limit Switches constantly
        boolean[] ballLimitArray = new boolean[] {Robot.drummag.getSlotOccuppied(1), Robot.drummag.getSlotOccuppied(2),
            Robot.drummag.getSlotOccuppied(3), Robot.drummag.getSlotOccuppied(4), Robot.drummag.getSlotOccuppied(5)};
        
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
        if (rotateToInfeed && (ballLimitArray[0] == false && ballLimitArray[4] == false)) {
            Robot.drummag.rotateToInfeed();
        }
        if (Robot.drummag.getMode() == false) { //If in 'facing infeed' mode, run through the method in 'DM' for filling
            Robot.drummag.infeedBalls();
        }

        // Bring Slot 1 to face Shooter through button
        if (rotateToShooter && (Robot.drummag.getMode() == false)) {
            Robot.drummag.rotateToShooter();
        }
        if (Robot.drummag.getMode() == true) { //if in 'facing shooter' mode, run through the method in 'DM' for emptying
            Robot.drummag.shootBalls();
        }

        //If shooting override is pressed, turn to face infeed and stop shooting process. 
            //Meant to ensure DrumMag and ballPuncher won't keep going when unwanted 
        if (cancelShooter && (Robot.drummag.getMode() == true)) { 
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
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

    // Constants
    int[] infeedSlots = new int[] { 0, 72, 144, 216, 288 }; // Pickup Setpoints (in degrees)
    int[] shooterSlots = new int[] { 180, 252, 324, 36, 108 }; // Shooting Setpoints (in degrees)

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

        // ***Logic for Drum Mag rotation based on conditions***//

        // Bring Slot 1 to face Infeed through button
        if (rotateToInfeed && (ballLimitArray[0] == false && ballLimitArray[4] == false)) {
            Robot.drummag.rotateToInfeed();
        }

        // Logic for determining when to turn to the next slot when infeeding
        
        //Runs through each value of the array for limit swithces, checking if each is 'true' through each iteration.
            //Once end is reached, and all balls are in, automatically switch to shooter
        if (Robot.drummag.getMode() == false) {
            for (int x = 0; x < infeedSlots.length; x++) {
                if (withinRange(Robot.drummag.getMagAngle(), infeedSlots[x]) && ballLimitArray[x]) {
                    if ((x + 1) < 5){
                        Robot.drummag.setPoint(shooterSlots[x + 1]);
                    }
                    else if (ballLimitArray[4] && ballLimitArray[0]){
                        Robot.drummag.rotateToShooter();
                    }
                }
            }
        }

        // Bring Slot 1 to face Shooter through button
        if (rotateToShooter && (Robot.drummag.getMode() == false)) {
            Robot.drummag.rotateToShooter();
        }

        // Logic for determining when to turn to the next slot when shooting
            
        //Runs through each value of the array for limit switches, checking if each is 'false' through each iteration. 
            //Once end is reached, and all balls are out, automatically switch back to infeed.
        if (Robot.drummag.getMode() == true) {  
            for (int x = 0; x < shooterSlots.length; x++) {
                if (withinRange(Robot.drummag.getMagAngle(), shooterSlots[x]) && ballLimitArray[x]) { //TODO: Make sure vision is targeting also
                    if (ballPunch) {
                        Robot.drummag.punchBall(); //punch ball 
                        //Add wait command to ensure ball is out(1 second for now)
                        if (!ballLimitArray[x]){                            
                            Robot.drummag.retractPuncher();
                            //Add wait command to allow ball to shoot (try 5 seconds). Don't want turning too early.
                            if ((x + 1) < 5) {
                                Robot.drummag.setPoint(shooterSlots[x + 1]);
                            }
                            else if (!ballLimitArray[4] && !ballLimitArray[0]) {
                                Robot.drummag.rotateToInfeed();
                            }
                        }
                    }                 
                }
            }
        }

            //If shooting override is pressed, turn to face infeed and stop shooting process. 
        //Meant to ensure DrumMag and ballPuncher won't keep going when unwanted 
        if (cancelShooter && (Robot.drummag.getMode() == true)) { 
            Robot.drummag.rotateToInfeed();
        }
    }

    //Checks to make sure the slot is within a decent range from the target angle
    private boolean withinRange(double actual, double target) {
        return Math.abs(actual - target) < (RobotMap.MAG_ERROR_TOLERANCE); //2 degrees
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
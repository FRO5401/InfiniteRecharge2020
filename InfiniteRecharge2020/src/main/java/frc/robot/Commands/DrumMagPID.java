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
    int[] shooterSlots = new int[] { 180, 252, 324, 36, 108 };

    boolean facingShooter;

    public DrumMagPID() {
        requires(Robot.drummag);

        facingShooter = false;

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.drummag.setPoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        // Infeed button
        boolean rotateToInfeed = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);

        // Shooter button
        boolean rotateToShooter = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_Y);

        // Read Limit Switches
        boolean[] ballLimitArray = new boolean[] { Robot.drummag.getSlotOccuppied(1), Robot.drummag.getSlotOccuppied(2),
                Robot.drummag.getSlotOccuppied(3), Robot.drummag.getSlotOccuppied(4),
                Robot.drummag.getSlotOccuppied(5) };

        // ***Logic for Drum Mag rotation based on conditions***//

        // Bring Slot 1 to face Infeed
        if (rotateToInfeed) {
            Robot.drummag.setPoint(infeedSlots[0]); // Set back to 0 degrees
            facingShooter = false;

        }

        // Logic for determining when to turn to the next slot when infeeding
        else if (!facingShooter) {
            for (int x = 0; x < infeedSlots.length; x++) {
                if (withinRange(Robot.drummag.getMagAngle(), infeedSlots[x], 4) && ballLimitArray[x]) {
                    if ((x + 1) < 5){
                        Robot.drummag.setPoint(shooterSlots[x + 1]);
                    }
                    else if (ballLimitArray[4]){
                        Robot.drummag.setPoint(shooterSlots[0]); // Set back to face shooter when all slots are full again
                        facingShooter = true;
                    }
                }
            }


            // Bring Slot 1 to face Shooter
            if (rotateToShooter) {
                Robot.drummag.setPoint(shooterSlots[0]); // Set to 180 degrees
                facingShooter = true;
            }

            // Logic for determining when to turn to the next slot when shooting
            if (facingShooter && (!ballLimitArray[4])) {
                Robot.drummag.setPoint(infeedSlots[0]); // Set back to face infeed when all slots are empty again
                facingShooter = false;
            }

            else if (facingShooter) {
                for (int x = 0; x < shooterSlots.length; x++) {
                    if (withinRange(Robot.drummag.getMagAngle(), shooterSlots[x], 4) && ballLimitArray[x]) {
                        Robot.drummag.ballPuncher.set(true);
                        if (!ballLimitArray[x]){
                            //Add wait command (1 second for now)
                            Robot.drummag.ballPuncher.set(false);
                            //Add wait command to allow ball to shoot (try 5 seconds)
                            if((x + 1) < 5) {
                                Robot.drummag.setPoint(shooterSlots[x + 1]);
                            }
                            else if(!ballLimitArray[4]) {
                                Robot.drummag.setPoint(infeedSlots[0]);
                                facingShooter = false;
                            }
                        }
                        else if (!ballLimitArray[4]){
                            Robot.drummag.setPoint(infeedSlots[0]); // Set back to face infeed when all slots are empty again
                            facingShooter = false;
                        }
                    }
                }
            }
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

    private boolean withinRange(double actual, double target, double error) {
        return Math.abs(actual - target) < error;

    }
}
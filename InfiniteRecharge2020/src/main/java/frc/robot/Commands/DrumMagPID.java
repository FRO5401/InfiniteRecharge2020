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
        
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        // Allows Operator to change between Infeed/Shooter
        boolean changeMagMode = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_B);

        boolean cellEjected = Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_A);

        if(changeMagMode){
            Robot.drummag.setMagMode();
        }

        int checkedLimit = Robot.drummag.getSlotPosition() - 1;

        if(Robot.drummag.getSlotPosition() < 5){
            if((Robot.drummag.getLimitPressed(checkedLimit) != Robot.drummag.getMagBoolean())
                && (Robot.drummag.cellEjectorSolenoid.get() == false)){
                    Robot.drummag.rotateOneSlot();
            }
        }
        else if (Robot.drummag.getSlotPosition() == 5){
            Robot.drummag.setMagMode();
        }

        // TODO: Check for Vision and RPM Flags OOP
        if(cellEjected){
            Robot.drummag.ejectSolenoid();
        }
        else{
            Robot.drummag.retractSolenoid();
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
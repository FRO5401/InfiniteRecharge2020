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

public class FeederControl extends Command {

    boolean controlFeeder;

    public FeederControl() {
        requires(Robot.feedersystem);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.feedersystem.runFeeder();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        controlFeeder = (Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_B));
        if(controlFeeder == true){
            Robot.feedersystem.runFeeder();
        }
        else{
            if(Robot.shooter.getVelocity() > 5000){
                Robot.feedersystem.runFeeder(); 
            }
            else{
                Robot.feedersystem.stopFeeder();
            }
        }
    } 

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.feedersystem.stopFeeder();
  }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.feedersystem.stopFeeder();
    }
}
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

public class BeltControl extends Command {

    boolean controlBelt;

    public BeltControl() {
        requires(Robot.beltchannel);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.beltchannel.runBelt();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        controlBelt = (Robot.oi.xboxButton(Robot.oi.xboxOperator, RobotMap.XBOX_BUTTON_B));
        if(controlBelt == true){
            Robot.beltchannel.runBelt();
        }
        else{
            if(Robot.shooter.getVelocity() > 5000){
                Robot.beltchannel.runBelt(); 
            }
            else{
                Robot.beltchannel.stopBelt();
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
        Robot.beltchannel.stopBelt();
  }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.beltchannel.stopBelt();
    }
}
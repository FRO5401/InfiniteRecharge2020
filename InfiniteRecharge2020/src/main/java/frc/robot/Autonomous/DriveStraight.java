/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.PowerCellSocket;
import frc.robot.Robot;

public class DriveStraight extends CommandGroup {
  /**
   * Add your docs here.
   */
  private double leftBound = 100;
  private double rightBound = 300;

  public DriveStraight() {
    double x = Robot.networktables.getXValue();
    while (x > leftBound && x < rightBound)
    {
      addSequential(new AutoDrive(50, 0.6));
      x = PowerCellSocket.getX();
      addSequential(new WaitCommand(0.5));
    }
    
    
    addSequential(new AutoDrive(0, 0.0));
    
    
    //addSequential(new AutoDrive(50, 0.6));
    //addSequential(new WaitCommand(0.5));
    //addSequential(new AutoDrive(-50, -0.6));
    //addSequential(new AutoDrive(0, 0.0));
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
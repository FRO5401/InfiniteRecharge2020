/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ShootBaselineNear extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ShootBaselineNear() {
    addSequential(new WaitCommand(0.01));
    addSequential(new AutoDrive(-30.0, -0.4));
    addSequential(new WaitCommand(0.3));
    //addSequential(new AutoTurret()); Shoots balls and targets
    //addSequential(new WaitCommand(0.5));
    addSequential(new AutoTurn(135.0, 0.3));
    addSequential(new WaitCommand(0.3));
    addSequential(new AutoDrive(87, 0.4));
    addSequential(new WaitCommand(0.3));
    addSequential(new AutoTurn(45, 0.3));

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

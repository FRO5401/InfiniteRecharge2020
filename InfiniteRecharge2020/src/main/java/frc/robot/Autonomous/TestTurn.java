/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class TestTurn extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TestTurn() {
    addSequential(new AutoTurn(-45.0, 0.3));
    //addSequential(new AutoTurn2017(-45.0, true, true));
    addSequential(new WaitCommand(1.0));
    addSequential(new AutoTurn(90.0, 0.3));
    //addSequential(new AutoTurn2017(90, true, false));
    addSequential(new WaitCommand(1.0));
    addSequential(new AutoTurn(-45.0, 0.3));
    //addSequential(new AutoTurn2017(-45.0, true, false));

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

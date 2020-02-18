/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.CommandTest;

/**
 * Add your docs here.
 */
public class SubsystemTest extends Subsystem {
  DigitalInput cell1, cell2, cell3, cell4, cell5;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public SubsystemTest(){
    cell1 = new DigitalInput(7); 
    cell2 = new DigitalInput(6);
    cell3 = new DigitalInput(5);
    cell4 = new DigitalInput(4);
    cell5 = new DigitalInput(3);
  }
  
  public void reportStuff(){
    SmartDashboard.putBoolean("Slot 1", cell1.get());
    SmartDashboard.putBoolean("Slot 2", cell2.get());
    SmartDashboard.putBoolean("Slot 3", cell3.get());
    SmartDashboard.putBoolean("Slot 4", cell4.get());
    SmartDashboard.putBoolean("Slot 5", cell5.get());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CommandTest());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

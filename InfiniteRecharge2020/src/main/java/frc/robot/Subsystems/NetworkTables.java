/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Commands.VisionMove;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class NetworkTables extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  NetworkTable table;
  NetworkTableInstance inst;
  NetworkTableEntry xEntry;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionMove());
    
  }

  public NetworkTables(){
    
  }

  public void getXValue(){
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SmartDashbooard");
    xEntry = table.getEntry("cX");
    
    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot
    try {
      Thread.sleep(1000);
    } catch (InterruptedException ex) {
      System.out.println("interrupted");
    }

    System.out.println(xEntry);
  
  }
}
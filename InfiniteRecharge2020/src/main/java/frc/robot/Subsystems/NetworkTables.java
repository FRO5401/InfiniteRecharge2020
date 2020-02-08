/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
//import frc.robot.Commands.VisionMove;
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
  NetworkTableEntry yEntry;
  private double x;
  private double y;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new VisionMove());
    
  }

  public NetworkTables(){
    
  }

  public double getXValue(){
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("data");
    xEntry = table.getEntry("X");
    yEntry = table.getEntry("Y");
    
    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot
    try {
      Thread.sleep(1000);
    } catch (InterruptedException ex) {
      System.out.println("interrupted");
    }

    x = xEntry.getDouble(0.0);
    y = yEntry.getDouble(0.0);
    System.out.println("X: " + x + " Y: " + y);
      
    return x;
  }
}
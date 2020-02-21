/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.VisionMove;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class NetworkTables extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  NetworkTable table;
  NetworkTableInstance inst;
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  private static double x;
  private static double y;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionMove());

  }

  public NetworkTables() {
    x = 0;
    y = 0;
  }

  public void updateValue() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SmartDashboard");
    xEntry = table.getEntry("cX");
    yEntry = table.getEntry("cY");

    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot
    

    x = xEntry.getDouble(0.0);
    y = yEntry.getDouble(0.0);
    System.out.println("X: " + x + " Y: " + y);
  }

  public double getXValue() {
    return x;
  }

  public double getYValue() {
    return y;
  }

  public void reportValues()
  {
    SmartDashboard.putNumber("Current X", getXValue());
    SmartDashboard.putNumber("Current Y", getYValue());
  }
}
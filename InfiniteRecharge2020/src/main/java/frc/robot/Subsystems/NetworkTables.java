/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class NetworkTables extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  NetworkTable ballTable;
  NetworkTable powerPortTable;
  NetworkTableInstance inst;
  NetworkTableEntry ballXEntry;
  NetworkTableEntry ballYEntry;
  NetworkTableEntry powerPortXEntry;
  NetworkTableEntry powerPortYEntry;
  private static double ballX;
  private static double ballY;
  private static double powerPortX;
  private static double powerPortY;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }

  public NetworkTables() {
    ballX = 0;
    ballY = 0;
    powerPortX = 0;
    powerPortY = 0;
  }

  public void updateValue() {
    inst = NetworkTableInstance.getDefault();
    ballTable = inst.getTable("SmartDashboard");
    powerPortTable = inst.getTable("PowerPort");
    ballXEntry = ballTable.getEntry("cX");
    ballYEntry = ballTable.getEntry("cY");
    powerPortXEntry = powerPortTable.getEntry("cX");
    powerPortYEntry = powerPortTable.getEntry("cY");

    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot
    

    ballX = ballXEntry.getDouble(0.0);
    ballY = ballYEntry.getDouble(0.0);
    powerPortX = powerPortXEntry.getDouble(0.0);
    powerPortY = powerPortYEntry.getDouble(0.0);
    System.out.println("The Ball coordinates are: " + "X: " + ballX + " Y: " + ballY);
    System.out.println("The Power Port coordinates are: " + "X: " + powerPortY + " Y: " + powerPortY);
  }

  public double getBXValue() {
    return ballX;
  }

  public double getBYValue() {
    return ballY;
  }

  public double getPPXValue(){
    return powerPortX;
  }

  public double getPPYValue(){
    return powerPortY;
  }

  public void reportValues()
  {
    SmartDashboard.putNumber("Current Ball X", getBXValue());
    SmartDashboard.putNumber("Current Ball Y", getBYValue());
    SmartDashboard.putNumber("Current Power Port X", getPPXValue());
    SmartDashboard.putNumber("Current Power Port Y", getPPYValue());
  }
}
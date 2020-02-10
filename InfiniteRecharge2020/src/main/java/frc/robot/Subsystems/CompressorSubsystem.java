/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;



public class CompressorSubsystem extends Subsystem {

  private final Compressor compressor;

  public CompressorSubsystem() {
    compressor = new Compressor(RobotMap.PCM_ID);
  }

  public void startCompressor() {
    compressor.setClosedLoopControl(true);
    compressor.start();
  }
  
  public void stopCompressor() {
    compressor.stop();
  }

  public void reportCompressorStatus() {
    SmartDashboard.putBoolean("Compressor Enabled", compressor.enabled());
    SmartDashboard.putBoolean("Compressor in Closed Looop", compressor.getClosedLoopControl());
    SmartDashboard.putNumber("Compressor Current Value", compressor.getCompressorCurrent());
    SmartDashboard.putBoolean("Compressor Pressure Switch On/Off", compressor.getPressureSwitchValue());
  }
  
  public boolean isEnabled() {
    return compressor.enabled();
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub

  }
}
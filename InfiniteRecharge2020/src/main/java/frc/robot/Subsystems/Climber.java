/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Solenoid climberLeft, climberRight;
  TalonSRX winchLeftSRX;
  TalonSRX winchRightSRX;

  private int loopIndex, slotIndex;

  public Climber(){
  //Speed Controller
  winchLeftSRX = new TalonSRX(RobotMap.WINCHLEFT_TALON_CHANNEL);   //put into robotmap
  winchRightSRX = new TalonSRX(RobotMap.WINCHRIGHT_TALON_CHANNEL);  

  //Solenoid
  climberLeft = new Solenoid(RobotMap.CLIMBERLEFT_CHANNEL);
  climberRight = new Solenoid(RobotMap.CLIMBERRIGHT_CHANNEL);

  winchLeftSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, loopIndex, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  // 10 is a timeout that waits for successful conection to sensor
  winchLeftSRX.setSensorPhase(true);
/*
  winchLeftSRX.configAllowableClosedloopError(slotIndex, RobotMap.MAGAZINE_THRESHOLD_FOR_PID,
      RobotMap.TIMEOUT_LIMIT_IN_Ms);
*/
  // Configuring the max & min percentage output.
  winchLeftSRX.configNominalOutputForward(0, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  winchLeftSRX.configNominalOutputReverse(0, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  winchLeftSRX.configPeakOutputForward(1, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  winchLeftSRX.configPeakOutputReverse(-1, RobotMap.TIMEOUT_LIMIT_IN_Ms);

  
  winchRightSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, loopIndex, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  // 10 is a timeout that waits for successful conection to sensor
  winchRightSRX.setSensorPhase(true);
/*
  winchRightSRX.configAllowableClosedloopError(slotIndex, RobotMap.MAGAZINE_THRESHOLD_FOR_PID,
      RobotMap.TIMEOUT_LIMIT_IN_Ms);
*/
  // Configuring the max & min percentage output.
  winchRightSRX.configNominalOutputForward(0, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  winchRightSRX.configNominalOutputReverse(0, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  winchRightSRX.configPeakOutputForward(1, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  winchRightSRX.configPeakOutputReverse(-1, RobotMap.TIMEOUT_LIMIT_IN_Ms);
}

  public void deploy(){ // Deploys both climbers
    climberLeft.set(true);
    climberRight.set(true);
  }

  public void setLeftWinchSpeed(double leftTrigger){
    winchLeftSRX.set(ControlMode.PercentOutput, leftTrigger);
  }

  public void setRightWinchSpeed(double leftTrigger){
    winchRightSRX.set(ControlMode.PercentOutput, leftTrigger);
  }

  public boolean getClimberStatus(){
    boolean status = false;
    if (climberLeft.get() && climberRight.get())
      status = true;
    return status;
  }

  public double getWinchSpeed(String winch){
    double winchSpeed = 0;
    if (winch.equals("Left"))
      winchSpeed = winchLeftSRX.getSelectedSensorVelocity() * RobotMap.CLIMBER_DPP; //TODO: find climber DPP
    if (winch.equals("Right"))
      winchSpeed = winchRightSRX.getSelectedSensorVelocity() * RobotMap.CLIMBER_DPP;
      return winchSpeed;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void reportClimberSensors() {
    SmartDashboard.putBoolean("Climbers Deployed", getClimberStatus());
    SmartDashboard.putNumber("Left Winch Speed", getWinchSpeed("Left"));
    SmartDashboard.putNumber("Right Winch Speed", getWinchSpeed("Right"));
  }
}

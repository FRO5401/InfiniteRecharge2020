/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Commands.FeederControl;

/**
 * Add your docs here.
 */
public class FeederSystem extends Subsystem {
  //2 motors, 4 solenoid
  //MIGHT NOT BE VICTOR SP, check with design
  private VictorSPX beltMotor, feederMotor1, feederMotor2;

  public FeederSystem(){
    beltMotor = new VictorSPX(RobotMap.BELT_MOTOR);
    feederMotor1 = new VictorSPX(RobotMap.FEEDER_MOTOR_1);
    feederMotor2 = new VictorSPX(RobotMap.FEEDER_MOTOR_2);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new FeederControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void runFeeder(){
    beltMotor.set(ControlMode.PercentOutput, RobotMap.FEEDER_SPEED);
    feederMotor1.set(ControlMode.PercentOutput, RobotMap.FEEDER_SPEED);
    feederMotor2.set(ControlMode.PercentOutput, -RobotMap.FEEDER_SPEED);
  }

  
  public void reverseFeeder(){
    beltMotor.set(ControlMode.PercentOutput, -1 * RobotMap.FEEDER_SPEED);
    feederMotor1.set(ControlMode.PercentOutput, -RobotMap.FEEDER_SPEED);
    feederMotor2.set(ControlMode.PercentOutput, RobotMap.FEEDER_SPEED);
  }

  public void stopFeeder(){
    beltMotor.set(ControlMode.PercentOutput, 0);
    feederMotor1.set(ControlMode.PercentOutput, 0);
    feederMotor2.set(ControlMode.PercentOutput, 0);
  }

}
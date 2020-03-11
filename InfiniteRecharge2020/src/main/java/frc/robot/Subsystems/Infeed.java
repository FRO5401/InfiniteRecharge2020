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
import frc.robot.Commands.InfeedControl;

/**
 * Add your docs here.
 */
public class Infeed extends Subsystem {
  //2 motors, 4 solenoid
  //MIGHT NOT BE VICTOR SP, check with design
  private VictorSPX infeedMotor1;
  private VictorSPX infeedMotor2;
  private VictorSPX infeedMotorFront;
  //private Solenoid deployInfeed;

  public Infeed(){
    infeedMotor1 = new VictorSPX(RobotMap.INFEED_MOTOR_LEFT);
    infeedMotor2 = new VictorSPX(RobotMap.INFEED_MOTOR_RIGHT);
    infeedMotorFront = new VictorSPX(RobotMap.CLIMB_MOTOR_1);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new InfeedControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void runInfeed(){
    infeedMotor1.set(ControlMode.PercentOutput, 0.75);
    infeedMotor2.set(ControlMode.PercentOutput, 0.75);
    infeedMotorFront.set(ControlMode.PercentOutput, 0.75);
  }

  
  public void reverseInfeed(){
    infeedMotor1.set(ControlMode.PercentOutput, -1 * RobotMap.INFEED_SPEED);
    infeedMotor2.set(ControlMode.PercentOutput, -1 * RobotMap.INFEED_SPEED);
    infeedMotorFront.set(ControlMode.PercentOutput, -1 * RobotMap.INFEED_SPEED);

  }

  public void stopInfeed(){
    infeedMotor1.set(ControlMode.PercentOutput, 0);
    infeedMotor2.set(ControlMode.PercentOutput, 0);
    infeedMotorFront.set(ControlMode.PercentOutput, 0);
  }

}
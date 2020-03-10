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
import frc.robot.Commands.BeltControl;

/**
 * Add your docs here.
 */
public class BeltChannel extends Subsystem {
  //2 motors, 4 solenoid
  //MIGHT NOT BE VICTOR SP, check with design
  private VictorSPX beltMotor1;
  private VictorSPX beltMotor2;

  public BeltChannel(){
    beltMotor1 = new VictorSPX(RobotMap.BELT_MOTOR_LEFT);
    beltMotor2 = new VictorSPX(RobotMap.BELT_MOTOR_RIGHT);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new BeltControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void runBelt(){
    beltMotor1.set(ControlMode.PercentOutput, RobotMap.BELT_SPEED);
    beltMotor2.set(ControlMode.PercentOutput, RobotMap.BELT_SPEED);
  }

  
  public void reverseBelt(){
    beltMotor1.set(ControlMode.PercentOutput, -1 * RobotMap.BELT_SPEED);
    beltMotor2.set(ControlMode.PercentOutput, -1 * RobotMap.BELT_SPEED);
  }

  public void stopBelt(){
    beltMotor1.set(ControlMode.PercentOutput, 0);
    beltMotor2.set(ControlMode.PercentOutput, 0);
  }

}
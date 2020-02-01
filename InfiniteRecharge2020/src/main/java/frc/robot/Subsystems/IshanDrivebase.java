/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Command;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;


public class IshanDrivebase extends Command {
  

  //motor

  private TalonSRX leftmotor1;
  private VictorSPX leftmotor2;
  private VictorSPX rightmotor2;
  private TalonSRX rightmotor1;
  private VictorSPX leftmotor3;
  private VictorSPX rightmotor3;


  //Sensors

  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private AHRS navxGyro;


  //gear shifter
  private Solenoid gearShifter;




  
  public IshanDrivebase() {
   

    //initializing motors
    leftmotor1 = new TalonSRX(RobotMap.DRIVE_MOTOR_LEFT_1); //value from robot map
    leftmotor2 = new VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_1);//value from robot map
    rightmotor1 = new TalonSRX(RobotMap.DRIVE_MOTOR_LEFT_2); //value from robot map
    rightmotor2 = new VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_2); 
    leftmotor3 = new VictorSPX(RobotMap.DRIVE_MOTOR_LEFT_3); 
    rightmotor3 = new VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_3);

    // initializing sensors
    navxGyro = new AHRS(I2C.Port.kMXP);
    leftEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
    rightEncoder = new Encoder(RobotMap.DRIVE_ENC_RIGHT_A, RobotMap.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);

    //initializing solenoid
    gearShifter = new Solenoid(0); //value from RobotMap


    //setting the desired speed


  }
  public void drive (double leftSpeedDesire, double rightSpeedDesire) {
    leftmotor1.set(ControlMode.PercentOutput, leftSpeedDesire);
    leftmotor2.set(ControlMode.PercentOutput, leftSpeedDesire);
    leftmotor3.set(ControlMode.PercentOutput, leftSpeedDesire);

    rightmotor1.set(ControlMode.PercentOutput, -1*rightSpeedDesire);
    rightmotor2.set(ControlMode.PercentOutput, -1*rightSpeedDesire);
    rightmotor3.set(ControlMode.PercentOutput, -1*rightSpeedDesire);



  }

  public void stopMotor() {
    leftmotor1.set(ControlMode.PercentOutput, 0);
    leftmotor2.set(ControlMode.PercentOutput, 0);
    rightmotor1.set(ControlMode.PercentOutput, 0);
    rightmotor2.set(ControlMode.PercentOutput, 0);


  }

  public void shiftGearLowtoHigh() {
    gearShifter.set(false);
    setDPPLowGear();


  }
 //DPP to high gear
  public void setDPPHighGear() {
    leftEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_RIGHT_DPP);

  }
  // Sets DPP for low gear.
  public void setDPPLowGear() {
    leftEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_RIGHT_DPP);
  }

// Reports all information from drivebase to SmartDashboard
public void reportDriveBaseSensors() {
  // Misc.
  SmartDashboard.putBoolean("NavX Connection", navxGyro.isConnected());
  SmartDashboard.putBoolean("DriveBase Current Gear", gearShifter.get());
  // Encoders
  SmartDashboard.putNumber("Left Enc Raw", leftEncoder.get());
  SmartDashboard.putNumber("Right Enc Raw", rightEncoder.get());
  SmartDashboard.putNumber("Left Enc Adj", leftEncoder.getDistance());
  SmartDashboard.putNumber("Right Enc Adj", rightEncoder.getDistance());
  // NavX
  SmartDashboard.putNumber("NaxX Angle", navxGyro.getAngle());
  SmartDashboard.putNumber("NavX Pitch", navxGyro.getPitch());
  SmartDashboard.putNumber("NavX Yaw", navxGyro.getYaw());
  // Victors
  SmartDashboard.putNumber("Left VSP1 Speed", leftmotor1.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Left VSP2 Speed", leftmotor2.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Right VSP1", rightmotor1.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Right VSP2", rightmotor2.getSelectedSensorVelocity());
}

 
 // Resets the Encoders.
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // Resets the Gyro.
  public void resetGyro() {
    navxGyro.reset();
  }

  @Override
  protected boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }


}

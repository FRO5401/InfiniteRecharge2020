/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Commands.XboxMove;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.VictorSP;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class DriveBase extends Subsystem {
  // Motors
  private TalonFX testPWM, falcon;

  private int maxVelocity = -14069;
  private int count = 1;
  private double initTime= 0;
  private double elapsedTime= 0;
  // Solenoids
  private Solenoid gearShifter;

  // Sensors
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private AHRS navxGyro;

  public DriveBase() {
    // Instantiate Motors
    testPWM = new TalonFX(19);
    falcon = new TalonFX(21);

    // Instantiate Solenoid.
    gearShifter = new Solenoid(RobotMap.GEAR_SHIFTER);

    // Instantiate Sensors
    navxGyro = new AHRS(I2C.Port.kMXP);
    leftEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
    rightEncoder = new Encoder(RobotMap.DRIVE_ENC_RIGHT_A, RobotMap.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new XboxMove());
  }


  // Sets victors to desired speed giving from XboxMove.
  public void drive() {
    testPWM.set(TalonFXControlMode.PercentOutput, -1 * 1);
    falcon.set(TalonFXControlMode.PercentOutput, 1);
  }

  public void stopMotors() {
    testPWM.set(TalonFXControlMode.PercentOutput, 0);
    falcon.set(TalonFXControlMode.PercentOutput, 0);
  }

  // Set shifter to low.
  public void shiftHighToLow() {
    gearShifter.set(true);
    setDPPLowGear();
  }

  // Set shifter to High.
  public void shiftLowToHigh() {
    gearShifter.set(false);
    setDPPHighGear();
  }

  // Sets DPP for low gear.
  public void setDPPLowGear() {
    leftEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_RIGHT_DPP);
  }

  // Sets DPP for high gear.
  public void setDPPHighGear() {
    leftEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_RIGHT_DPP);
  }

  // Gets Gyro Angle
  public double getGyroAngle() {
    return navxGyro.getAngle();
  }

  public double getGyroPitch() {
    double pitch = navxGyro.getPitch();
    return pitch;
  }

  // Resets the Gyro.
  public void resetGyro() {
    navxGyro.reset();
  }

  public void reportShooter() {
    SmartDashboard.putNumber("max speed", maxVelocity * -1);
    SmartDashboard.putNumber("Speed", -1*testPWM.getSelectedSensorVelocity());
    SmartDashboard.putNumber("recoverTime", elapsedTime);
    SmartDashboard.putNumber("count",count);
  }

 /* public void maxVelocity() {
    if(testPWM.getSelectedSensorVelocity() < maxVelocity){
      maxVelocity = testPWM.getSelectedSensorVelocity();

    }
   


  } 
  */
  public void reportTimer(){ 
    if(testPWM.getSelectedSensorVelocity() <  maxVelocity + 10 && testPWM.getSelectedSensorVelocity() > maxVelocity - 10){
      
       
      if((count % 3) == 0 && (testPWM.getSelectedSensorVelocity() <  maxVelocity + 10 && testPWM.getSelectedSensorVelocity() > maxVelocity - 10)){
         elapsedTime = initTime - Timer.getMatchTime();
         count = 1;
      }
      else{
        initTime = Timer.getMatchTime();
      }
      count++;
      
    }


  }


}


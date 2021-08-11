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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class DriveBase extends Subsystem {
  // Motors
  private TalonSRX leftDrive1;
  private TalonSRX rightDrive1;
  private VictorSPX leftDrive2;
  private VictorSPX rightDrive2;
  private VictorSPX leftDrive3;
  private VictorSPX rightDrive3;

  //PID stuff
  private int loopIndex, slotIndex;
  private double DRIVEBASE_kF = 0;
  private double DRIVEBASE_kP = 0;
  private double DRIVEBASE_kI = 0;
  private double DRIVEBASE_kD = 0;

  private int iaccum = 0;

  // Solenoids
  private Solenoid gearShifter;

  // Sensors
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private AHRS navxGyro;

  public DriveBase() {

    loopIndex = 0;
    slotIndex = 0;

    // Instantiate Motors
    leftDrive1 = new TalonSRX(RobotMap.DRIVE_MOTOR_LEFT_1);
    rightDrive1 = new TalonSRX(RobotMap.DRIVE_MOTOR_RIGHT_1);
    leftDrive2 = new VictorSPX(RobotMap.DRIVE_MOTOR_LEFT_2);
    rightDrive2 = new VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_2);
    leftDrive3 = new VictorSPX(RobotMap.DRIVE_MOTOR_LEFT_3);
    rightDrive3 = new VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_3);

    // Instantiate Solenoid.
    gearShifter = new Solenoid(RobotMap.GEAR_SHIFTER);

    // Instantiate Sensors
    navxGyro = new AHRS(I2C.Port.kMXP);
    leftEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
    rightEncoder = new Encoder(RobotMap.DRIVE_ENC_RIGHT_A, RobotMap.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);

      //Configuring PID values
    leftDrive2.follow(leftDrive1);
    leftDrive3.follow(leftDrive1);
    

    rightDrive2.follow(rightDrive1);
    rightDrive3.follow(rightDrive1);
    

    leftDrive1.setNeutralMode(NeutralMode.Brake);
    rightDrive1.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new XboxMove());
  }

  // Sets victors to desired speed giving from XboxMove.
  public void drive(double leftDriveDesired, double rightDriveDesired) {
    // Left inverted in accordance to physical wiring.
    leftDrive1.set(ControlMode.PercentOutput, -1* leftDriveDesired * RobotMap.TELEOP_SPEED_ADJUSTMENT_LEFT);
    rightDrive1.set(ControlMode.PercentOutput, (rightDriveDesired) * RobotMap.TELEOP_SPEED_ADJUSTMENT_RIGHT);
  }

  // Sets SC's to 0.
  public void stopMotors() {
    leftDrive1.set(ControlMode.PercentOutput, 0);
    rightDrive1.set(ControlMode.PercentOutput, 0);
  }

  //PID control during Teleop
  public void driveToPosition(double distance) {
    double leftDist = distance * 16384;
    double rightDist = distance * 16384;
    
    // Left inverted in accordance to physical wiring.
    leftDrive1.set(ControlMode.MotionMagic, leftDist);
    leftDrive1.set(ControlMode.Position, leftDist);
    rightDrive1.set(ControlMode.MotionMagic, rightDist);
    rightDrive1.set(ControlMode.Position, rightDist);
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

  // For autonomous driving
  public double getEncoderDistance(int encoderNumber) {
    double leftDistAdj = leftEncoder.getDistance();
    double rightDistAdj = rightEncoder.getDistance();
    double avgDistance = (leftDistAdj + rightDistAdj) / 2;

    if (encoderNumber == 1) {
      return leftDistAdj;
    } else if (encoderNumber == 2) {
      return rightDistAdj;
    } else {
      return avgDistance;
    }
  }

  // Gets Gyro Angle
  public double getGyroAngle() {
    return navxGyro.getAngle();
  }

  public double getGyroPitch() {
    double pitch = navxGyro.getPitch();
    return pitch;
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
    SmartDashboard.putNumber("Left Talon1 Position", leftDrive1.getSelectedSensorPosition());
    //SmartDashboard.putNumber("Left VSP2 Speed", leftDrive2.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("Left VSP3 Speed", leftDrive3.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Talon1 Position", rightDrive1.getSelectedSensorPosition());
    //SmartDashboard.putNumber("Right VSP2 Speed", rightDrive2.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("Right VSP3 Speed", rightDrive3.getSelectedSensorVelocity());
  }

  // Resets the Encoders.
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();

    leftDrive1.getSensorCollection().setQuadraturePosition(0, 10);
    rightDrive1.getSensorCollection().setQuadraturePosition(0, 10);
  }

  // Resets the Gyro.
  public void resetGyro() {
    navxGyro.reset();
  }
}
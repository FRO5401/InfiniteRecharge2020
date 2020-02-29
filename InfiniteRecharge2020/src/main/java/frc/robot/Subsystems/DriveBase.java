/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Commands.XboxMove;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
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

  // Solenoids
  private Solenoid gearShifter;

  // Sensors
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  public AHRS navxGyro;

  public DriveBase() {
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
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new XboxMove());
  }

  // Sets victors to desired speed giving from XboxMove.
  public void autoDrive(double leftDriveDesired, double rightDriveDesired, double angle) {
    if (leftDriveDesired > 0 && rightDriveDesired > 0){ //driving forwards
      if (angle > 0){ //drifting right
        leftDrive1.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive2.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive3.set(ControlMode.PercentOutput, leftDriveDesired);
        rightDrive1.set(ControlMode.PercentOutput, -1 * rightDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        rightDrive2.set(ControlMode.PercentOutput, -1 * rightDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        rightDrive3.set(ControlMode.PercentOutput, -1 * rightDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
      }
      else if (angle < 0){ //drifting left
        leftDrive1.set(ControlMode.PercentOutput, leftDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        leftDrive2.set(ControlMode.PercentOutput, leftDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        leftDrive3.set(ControlMode.PercentOutput, leftDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        rightDrive1.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive2.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive3.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
      } 
      else{
        leftDrive1.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive2.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive3.set(ControlMode.PercentOutput, leftDriveDesired);
        rightDrive1.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive2.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive3.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
      }
    }
    else if (leftDriveDesired < 0 && rightDriveDesired < 0){ //driving backwards
      if (angle > 0){ //drifting right
        leftDrive1.set(ControlMode.PercentOutput, leftDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        leftDrive2.set(ControlMode.PercentOutput, leftDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        leftDrive3.set(ControlMode.PercentOutput, leftDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        rightDrive1.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive2.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive3.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
      }
      else if (angle < 0){ //drifting left
        leftDrive1.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive2.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive3.set(ControlMode.PercentOutput, leftDriveDesired);
        rightDrive1.set(ControlMode.PercentOutput, -1 * rightDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        rightDrive2.set(ControlMode.PercentOutput, -1 * rightDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
        rightDrive3.set(ControlMode.PercentOutput, -1 * rightDriveDesired * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.05);
      } 
      else{
        leftDrive1.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive2.set(ControlMode.PercentOutput, leftDriveDesired);
        leftDrive3.set(ControlMode.PercentOutput, leftDriveDesired);
        rightDrive1.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive2.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
        rightDrive3.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
      }
    }
    else{ //When leftDrive1 and rightDrive1 are zero
      stopMotors();      
    }
  }
  public void autoTurn(double desiredAngle, double autoTurnSpeed) {
    navxGyro.reset();

    double angleTraveled;
    boolean doneTraveling = false;
    
    angleTraveled = getGyroAngle();
    if ((angleTraveled) <= (desiredAngle) && desiredAngle > 0){
			drive(autoTurnSpeed, (-1 * autoTurnSpeed));
			doneTraveling = false;
		}
		else if(angleTraveled >= (desiredAngle) && desiredAngle < 0){
      drive((-1 * autoTurnSpeed), autoTurnSpeed);
      doneTraveling = false;
		}
		else{
			stopMotors();
			doneTraveling = true;
    }
  }

  public void drive(double leftDriveDesired, double rightDriveDesired) {
    leftDrive1.set(ControlMode.PercentOutput, leftDriveDesired);
    leftDrive2.set(ControlMode.PercentOutput, leftDriveDesired);
    leftDrive3.set(ControlMode.PercentOutput, leftDriveDesired);
    rightDrive1.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
    rightDrive2.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
    rightDrive3.set(ControlMode.PercentOutput, -1 * rightDriveDesired);
  }

  // Sets SC's to 0.
  public void stopMotors() {
    leftDrive1.set(ControlMode.PercentOutput, 0);
    leftDrive2.set(ControlMode.PercentOutput, 0);
    leftDrive3.set(ControlMode.PercentOutput, 0);
    rightDrive1.set(ControlMode.PercentOutput, 0);
    rightDrive2.set(ControlMode.PercentOutput, 0);
    rightDrive3.set(ControlMode.PercentOutput, 0);
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
    double leftDistAdj = leftDrive1.getSelectedSensorPosition();
    double rightDistAdj = rightDrive1.getSelectedSensorPosition();
    double avgDistance = ((-1 * leftDistAdj) + rightDistAdj) / 2;

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

  //resets sensors 
  public void resetSensors() {
    leftEncoder.reset();
    rightEncoder.reset();

    leftDrive1.getSensorCollection().setQuadraturePosition(0, 10);
    rightDrive1.getSensorCollection().setQuadraturePosition(0, 10);

    //Robot.turret.turretTalon.getSensorCollection().setQuadraturePosition(0, 10);

    navxGyro.reset();
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
    SmartDashboard.putNumber("Left VSP2 Speed", leftDrive2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left VSP3 Speed", leftDrive3.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Talon1 Position", rightDrive1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right VSP2 Speed", rightDrive2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right VSP3 Speed", rightDrive3.getSelectedSensorVelocity());
  }
}
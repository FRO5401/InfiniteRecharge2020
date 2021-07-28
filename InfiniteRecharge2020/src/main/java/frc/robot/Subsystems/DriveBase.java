/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro = new AHRS(I2C.Port.kMXP);

  // Motors
  private SpeedControllerGroup leftDrives = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.DRIVE_MOTOR_LEFT_1), new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_LEFT_2), new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_LEFT_3));
  private SpeedControllerGroup rightDrives = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.DRIVE_MOTOR_RIGHT_1), new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_2), new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_3));
  private DifferentialDrive ourDrive = new DifferentialDrive(leftDrives, rightDrives);
  private DifferentialDriveOdometry odometry;

  //PID stuff
  private int loopIndex, slotIndex;
  private double DRIVEBASE_kF = 0;
  private double DRIVEBASE_kP = 0;
  private double DRIVEBASE_kI = 0;
  private double DRIVEBASE_kD = 0;

  private int iaccum = 0;

  // Solenoids
  private Solenoid gearShifter = new Solenoid(RobotMap.GEAR_SHIFTER);

  // Sensors
  private Encoder leftEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
  private Encoder rightEncoder = new Encoder(RobotMap.DRIVE_ENC_RIGHT_A, RobotMap.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);
  

  public DriveBase() {

    rightDrives.setInverted(true);

    leftEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_RIGHT_DPP);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d());
    
  }

  @Override
  public void periodic() {
    odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }


  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, navxGyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    ourDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrives.setVoltage(leftVolts);
    rightDrives.setVoltage(-rightVolts);
    ourDrive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }



  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    ourDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navxGyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -navxGyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navxGyro.getRate();
  }

  
  
}
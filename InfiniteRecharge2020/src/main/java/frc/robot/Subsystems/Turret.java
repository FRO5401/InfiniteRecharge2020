/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Commands.*;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Turret extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Creates variables for PID, threshold, centerpoint on camera, and angle
  // setpoints
  private int loopIndex, slotIndex;
  private final double CENTERPOINT = 0;
  private double TURRET_kF = 0;
  private double TURRET_kP = 0;
  private double TURRET_kI = 0;
  private double TURRET_kD = 0;
  private double turretAngle = 45;
  private double resetAngle = 0;
  private int TIMEOUT_LIMIT_MS = 10;
  private int TURRET_PID_THRESHOLD = 2;
  private double targetLocation;
  private boolean turretPidEnabled;

  // Creates the actual robot parts
  TalonSRX turretTalon;

  // Constructor
  public Turret() {
    // Creates instances of the actual robot parts
    turretTalon = new TalonSRX(RobotMap.TURRET_TALON);

    loopIndex = 0;
    slotIndex = 0;

    turretTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, loopIndex, TIMEOUT_LIMIT_MS);
    turretTalon.configAllowableClosedloopError(slotIndex, TURRET_PID_THRESHOLD, TIMEOUT_LIMIT_MS);

    // Setting Max and Min values.
    turretTalon.configNominalOutputForward(0, TIMEOUT_LIMIT_MS);
    turretTalon.configNominalOutputReverse(0, TIMEOUT_LIMIT_MS);
    turretTalon.configPeakOutputForward(1, TIMEOUT_LIMIT_MS);
    turretTalon.configPeakOutputReverse(-1, TIMEOUT_LIMIT_MS);

    // Configurating the actual PID values
    turretTalon.config_kF(slotIndex, TURRET_kF, TIMEOUT_LIMIT_MS);
    turretTalon.config_kP(slotIndex, TURRET_kP, TIMEOUT_LIMIT_MS);
    turretTalon.config_kI(slotIndex, TURRET_kI, TIMEOUT_LIMIT_MS);
    turretTalon.config_kD(slotIndex, TURRET_kD, TIMEOUT_LIMIT_MS);
  }

  public void turretSetTalonNeutralMode(NeutralMode neutralMode) {
    turretTalon.setNeutralMode(neutralMode);
    SmartDashboard.putString("Neutral Mode", neutralMode.toString());
  }

  public void setPoint(double setPoint) {
    double setPointNativeUnits = setPoint / RobotMap.TURRET_ANGLE_PER_PULSE;
    turretTalon.set(ControlMode.Position, setPointNativeUnits);
  }

  // Will set the turret angle to its original position
  public void resetTurretAngle() {
    // Reset to 0 degrees (default)
    turretTalon.set(ControlMode.Position, resetAngle);
  }

  // Will await data from the network tables
  public void setTargetLocation(double targetLocation) {
    this.targetLocation = targetLocation;
  }

  //I don't know if this is necessary
  public boolean onTarget() {
    boolean onTarget = Math.abs(turretTalon.getSensorCollection().getQuadraturePosition()
        - turretTalon.getClosedLoopTarget(loopIndex)) < RobotMap.ANGLE_THRESHOLD;

    return onTarget;
  }

  /*
   * This method will calculate the distance from the current point and center
   * point. Then it will check if its on target. After, it will move in a certain
   * direction depending on if the distance is negative or positive.
   */

  // Still need help here.
  public void readyTurret() {
    double position = turretTalon.getSensorCollection().getQuadraturePosition();
    // solve for a.p.p later
    turretPidEnabled = true;
    double distance = targetLocation - CENTERPOINT;
    setPoint(position + distance);
  }

  // Will set the motor such that the turret rotates left
  public void rotateTurretLeft() {
    turretTalon.set(ControlMode.Velocity, RobotMap.TURRET_TURN_SPEED);
  }

  // Will set the motor such that the turret rotates right
  public void rotateTurretRight() {
    turretTalon.set(ControlMode.Velocity, RobotMap.TURRET_TURN_SPEED * -1);
  }

  // Will end the rotation of the turret motor
  public void stopRotation() {
    turretTalon.set(ControlMode.Velocity, 0);
  }

  // Override the movement
  public void overrideTurret(double joystickSpeed) {
    turretPidEnabled = false;
    joystickSpeed *= (-1 * RobotMap.TURRET_SPEED_SENSITIVITY);
    turretTalon.set(ControlMode.PercentOutput, joystickSpeed);
  }

  // Will find the current angle of the turret
  // TODO
  public double getTurretAngle() {
    turretAngle = turretTalon.getSensorCollection().getQuadraturePosition();
    return turretAngle;
  }

  // Creates a left limit
  public boolean getLimitLeft() {
    if (getTurretAngle() < RobotMap.T_STOP_LEFT) {
      return true;
    } else {
      return false;
    }
  }

  // Creates a right limit
  public boolean getLimitRight() {
    if (getTurretAngle() > RobotMap.T_STOP_RIGHT) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TurretTurn());
  }

  // Will report the necessary data to shuffleboard/
  public void reportTurretInfeedSensors() {
    SmartDashboard.putBoolean("Left Limit Turret", getLimitLeft());
    SmartDashboard.putBoolean("Right Limit Turret", getLimitRight());
    SmartDashboard.putNumber("Turret Direction", turretTalon.getSelectedSensorVelocity());
  }
}

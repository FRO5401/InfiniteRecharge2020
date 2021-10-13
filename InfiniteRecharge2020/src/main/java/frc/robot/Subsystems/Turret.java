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
import frc.robot.Robot;
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
  private double TURRET_kP = 1;
  private double TURRET_kI = 0;
  private double TURRET_kD = 0;
  private double turretAngle = 45;
  private double resetAngle = 0;
  private int TIMEOUT_LIMIT_MS = 10;
  private int TURRET_PID_THRESHOLD = (int) (1.0 * RobotMap.TURRET_ANGLE_PER_PULSE);
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


  // Will end the rotation of the turret motor
  public void stopRotation() {
    turretTalon.set(ControlMode.PercentOutput, 0);
  }

  // Reset to 0 degrees (default)
  public void resetTurretAngle() {
    goToAngle(resetAngle);
  }
  
  // Enables manual control
  public void moveTurret(double joystickSpeed) {
    if(getLimitLeft() == false && getLimitRight() == false){
      joystickSpeed *= RobotMap.TURRET_SPEED_SENSITIVITY;
      turretTalon.set(ControlMode.PercentOutput, joystickSpeed);
    }
    else if(getLimitRight() == true || getLimitLeft() == true){
      joystickSpeed *= RobotMap.TURRET_SPEED_SENSITIVITY;
      turretTalon.set(ControlMode.PercentOutput, joystickSpeed * -1);
    }
    else{
      stopRotation();
    }
  }

  // Creates a left limit
  public boolean getLimitLeft() {
    if (getTurretAngle() <=  (RobotMap.TURRET_ANGLE_LIMIT * -1)) {
      return true;
    } else {
      return false;
    }
  }

  // Creates a right limit
  public boolean getLimitRight() {
    if (getTurretAngle() >= RobotMap.TURRET_ANGLE_LIMIT) {
      return true;
    } else {
      return false;
    }
  }


    // Uses A.P.P and does the math to go to a certain position
  public void goToAngle(double desiredAngle){
    turretTalon.set(ControlMode.Position, desiredAngle * RobotMap.TURRET_ANGLE_PER_PULSE); 
  }

  // Will find the current angle of the turret
  public double getTurretAngle() {
    turretAngle = turretTalon.getSensorCollection().getQuadraturePosition() / RobotMap.TURRET_ANGLE_PER_PULSE;
    return turretAngle;
  }

  // Sets the mode of the turret
  public void turretSetTalonNeutralMode(NeutralMode neutralMode) {
    turretTalon.setNeutralMode(neutralMode);
    SmartDashboard.putString("Neutral Mode", neutralMode.toString());
  }


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TurretControl());
  }

  // Will report the necessary data to shuffleboard
  public void reportTurretInfeedSensors() {
    SmartDashboard.putBoolean("Left Limit Turret", getLimitLeft());
    SmartDashboard.putBoolean("Right Limit Turret", getLimitRight());
    SmartDashboard.putNumber("Turret Direction", turretTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Angle", getTurretAngle());
  }
}
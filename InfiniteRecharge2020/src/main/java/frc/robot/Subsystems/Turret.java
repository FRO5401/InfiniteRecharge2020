/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Turret extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private int loopIndex, slotIndex;

  

  private double TURRET_kF = 0;
  private double TURRET_kP = 0;
  private double TURRET_kI = 0;
  private double TURRET_kD = 0;

  private double turretAngle = 45; 
  private double resetPosition = 0;

  private int TIMEOUT_LIMIT_MS = 10;
  private int TURRET_PID_THRESHOLD = 2;

  VictorSP turretMotor;
  TalonSRX turretTalon;
  DigitalInput turretLimitLeft, turretLimitRight;

  public Turret () {
    turretMotor = new VictorSP(RobotMap.TURRET_MOTOR);
    turretTalon = new TalonSRX(RobotMap.TURRET_TALON);
    turretLimitLeft = new DigitalInput(RobotMap.T_STOP_LEFT);
    turretLimitRight = new DigitalInput(RobotMap.T_STOP_RIGHT);

    loopIndex = 0;
    slotIndex = 0;

    turretTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, loopIndex, TIMEOUT_LIMIT_MS);
    turretTalon.configAllowableClosedloopError(slotIndex, TURRET_PID_THRESHOLD, TIMEOUT_LIMIT_MS);
    
      //Setting Max and Min values. 
    turretTalon.configNominalOutputForward(0, TIMEOUT_LIMIT_MS);
    turretTalon.configNominalOutputReverse(0, TIMEOUT_LIMIT_MS);
    turretTalon.configPeakOutputForward(1, TIMEOUT_LIMIT_MS);
    turretTalon.configPeakOutputReverse(-1, TIMEOUT_LIMIT_MS);

    turretTalon.config_kF(slotIndex, TURRET_kF, TIMEOUT_LIMIT_MS);
    turretTalon.config_kP(slotIndex, TURRET_kP, TIMEOUT_LIMIT_MS);
    turretTalon.config_kI(slotIndex, TURRET_kI, TIMEOUT_LIMIT_MS);
    turretTalon.config_kD(slotIndex, TURRET_kD, TIMEOUT_LIMIT_MS);   
  }

  public void resetTurretPosition(){
      turretTalon.set(ControlMode.Position, resetPosition);
    }

  public void rotateTurretLeft() {
      turretMotor.set(RobotMap.TURRET_TURN_SPEED);
  }  

  public void rotateTurretRight() {
      turretMotor.set(RobotMap.TURRET_TURN_SPEED * -1);
  }

  public void stopRotation() {
      turretMotor.set(0);
  }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}


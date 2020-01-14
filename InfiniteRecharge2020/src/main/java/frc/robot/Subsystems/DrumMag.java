/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

//import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DrumMag extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX magazineSRX;

  DigitalInput ballLimit1, ballLimit2, ballLimit3, ballLimit4, ballLimit5;

  private boolean magazinePidEnabled;
  private int loopIndex, slotIndex;

  private double magazine_kF = 0;
  private double magazine_kP = 0;
  private double magazine_kI = 0;
  private double magazine_kD = 0;

  // FIND LATER
  public double MAGAZINE_ANGLE_PER_PULSE = 0;

  public DrumMag() {

    loopIndex = 0;
    slotIndex = 0;

    magazineSRX = new TalonSRX(RobotMap.MAGAZINE_TALON_CHANNEL);

    // Limits
    ballLimit1 = new DigitalInput(RobotMap.REVOLVER_STOP_1);
    ballLimit2 = new DigitalInput(RobotMap.REVOLVER_STOP_2);
    ballLimit3 = new DigitalInput(RobotMap.REVOLVER_STOP_3);
    ballLimit4 = new DigitalInput(RobotMap.REVOLVER_STOP_4);
    ballLimit5 = new DigitalInput(RobotMap.REVOLVER_STOP_5);

    magazineSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, loopIndex, RobotMap.TIMEOUT_LIMIT_IN_Ms);
    // 10 is a timeout that waits for successful conection to sensor
    magazineSRX.setSensorPhase(true);

    magazineSRX.configAllowableClosedloopError(slotIndex, RobotMap.MAGAZINE_THRESHOLD_FOR_PID,
        RobotMap.TIMEOUT_LIMIT_IN_Ms);

    // Configuring the max & min percentage output.
    magazineSRX.configNominalOutputForward(0, RobotMap.TIMEOUT_LIMIT_IN_Ms);
    magazineSRX.configNominalOutputReverse(0, RobotMap.TIMEOUT_LIMIT_IN_Ms);
    magazineSRX.configPeakOutputForward(1, RobotMap.TIMEOUT_LIMIT_IN_Ms);
    magazineSRX.configPeakOutputReverse(-1, RobotMap.TIMEOUT_LIMIT_IN_Ms);

    // Configuring PID values.
    magazineSRX.config_kF(slotIndex, magazine_kF, RobotMap.TIMEOUT_LIMIT_IN_Ms);
    magazineSRX.config_kP(slotIndex, magazine_kP, RobotMap.TIMEOUT_LIMIT_IN_Ms);
    magazineSRX.config_kI(slotIndex, magazine_kI, RobotMap.TIMEOUT_LIMIT_IN_Ms);
    magazineSRX.config_kD(slotIndex, magazine_kD, RobotMap.TIMEOUT_LIMIT_IN_Ms);
  }

  @Override
  public void initDefaultCommand() {
  }
   

  // magazine Stopped with PID/Interrupted
  public void magazineStop() {
    magazinePidEnabled = false;
  }

  // Sets the point to which the magazine will move
  public void setPoint(double setPoint) {
    double setPointNativeUnits = setPoint / MAGAZINE_ANGLE_PER_PULSE;
    magazineSRX.set(ControlMode.Position, setPointNativeUnits);
    magazinePidEnabled = true;
  }

  // Sets the NeutralMode of the magazine (BRAKE or COAST)
  public void setMagazineNeutralMode(NeutralMode neutralMode) {
    magazineSRX.setNeutralMode(neutralMode);
  }

  public boolean onTarget() {
    // Method returns true if on target
    boolean onTarget = Math.abs(magazineSRX.getSensorCollection().getQuadraturePosition()
        - magazineSRX.getClosedLoopTarget(loopIndex)) < RobotMap.MAGAZINE_THRESHOLD_FOR_PID;
    return onTarget;
    // getClosedLoopT gets the SetPoint already set (or moving to)
  }

  // Potential limit switch for Magazine rotation
  public boolean getSlotOccuppied(int slot) {

    boolean status = false;

    switch (slot) {
    case 1:
      status = ballLimit1.get();
      break;
    case 2:
      status = ballLimit2.get();
      break;
    case 3:
      status = ballLimit3.get();
      break;
    case 4:
      status = ballLimit4.get();
      break;
    case 5:
      status = ballLimit5.get();
      break;
    default:
      System.out.print("Doin yourmom");
    }

    return status;
  }

  // probably not right
  public double getMagAngle() {
    return (magazineSRX.getSensorCollection().getQuadraturePosition() * MAGAZINE_ANGLE_PER_PULSE);
  }

  public void reportElevatorSensors() {
    SmartDashboard.putBoolean("Slot 1 Status", getSlotOccuppied(1));
    SmartDashboard.putBoolean("Slot 2 Status", getSlotOccuppied(2));
    SmartDashboard.putBoolean("Slot 3 Status", getSlotOccuppied(3));
    SmartDashboard.putBoolean("Slot 4 Status", getSlotOccuppied(4));
    SmartDashboard.putBoolean("Slot 5 Status", getSlotOccuppied(5));
    SmartDashboard.putNumber("Current Angle (Raw)", getMagAngle());
    SmartDashboard.putNumber("Current Slot", (getMagAngle() / 72) +1);
    /* 
      Slot that would be facing either infeed or shooter. 
      **If on integer from 1-5, will be facing infeed
      **If one 0.5 value, will be facing shooter
    Angle/72 because total angle of circle is 360, and there are 5 slots, +1 to account for physical position.

    */
  }
}

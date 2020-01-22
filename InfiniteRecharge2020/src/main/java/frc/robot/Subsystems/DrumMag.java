/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class DrumMag extends Subsystem {

  TalonSRX magazineSRX;
  public Solenoid cellEjectorSolenoid;

  public DigitalInput cellLimit1, cellLimit2, cellLimit3, cellLimit4, cellLimit5;

  private int loopIndex, slotIndex;
  public String magMode;

  private double magazine_kF = 0;
  private double magazine_kP = 0;
  private double magazine_kI = 0;
  private double magazine_kD = 0;
  private double infeedPositionPID, shooterPositionPID;

  //TODO: GOES INTO ROBOTMAP
  double nativeUnitsForOneCell;

  public DrumMag() {

    loopIndex = 0;
    slotIndex = 0;

    //Speed controller for the drum mag
    magazineSRX = new TalonSRX(RobotMap.MAGAZINE_TALON_CHANNEL);

    //Solenoid
    cellEjectorSolenoid = new Solenoid(RobotMap.MAGAZINE_CELL_EJECTOR_CHANNEL);

    // Limits
    cellLimit1 = new DigitalInput(RobotMap.MAGAZINE_STOP_1);
    cellLimit2 = new DigitalInput(RobotMap.MAGAZINE_STOP_2);
    cellLimit3 = new DigitalInput(RobotMap.MAGAZINE_STOP_3);
    cellLimit4 = new DigitalInput(RobotMap.MAGAZINE_STOP_4);
    cellLimit5 = new DigitalInput(RobotMap.MAGAZINE_STOP_5);

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

  // Sets the point to which the magazine will move
  public void setPoint(double setPoint) {
    double setPointNativeUnits = setPoint / RobotMap.MAGAZINE_ANGLE_PER_PULSE;
    magazineSRX.set(ControlMode.Position, setPointNativeUnits);
  }

  public boolean onTarget() {
    // Method returns true if on target
    boolean onTarget = Math.abs(magazineSRX.getSensorCollection().getQuadraturePosition()
        - magazineSRX.getClosedLoopTarget(loopIndex)) < RobotMap.MAGAZINE_THRESHOLD_FOR_PID;
    return onTarget;
    // getClosedLoopT gets the desired angle
  }

  public void ejectSolenoid(){
    cellEjectorSolenoid.set(true);
  }

  public void retractSolenoid(){
    cellEjectorSolenoid.set(false);
  }

    // Changes mode from shooter to infeed or infeed to shooter, and 
  public void setMagMode(){
    if(magMode.equals("infeed")){
      magMode = "shooter";
      setPoint(shooterPositionPID);
    }
    
    if(magMode.equals("shooter")){
      magMode = "infeed";
      setPoint(infeedPositionPID);
    }
  }

  public String getMagMode(){
    return magMode;
  }

  public void rotateOneSlot(){
    double position = magazineSRX.getSensorCollection().getQuadraturePosition();
    setPoint(position + nativeUnitsForOneCell);
  }

  //Tells Operator amount of ammo
  public int getSlotPosition(){
     
    int slotPosition = 0;

    boolean status = false;

    if(magMode.equals("infeed")){
      status = false;
    } else if (magMode.equals("shooter")){
      status = true;
    }

    if(cellLimit1.get() == status){
      slotPosition = 1;
    }
    else if(cellLimit2.get() == status){
      slotPosition = 2;
    }
    else if(cellLimit3.get() == status){
      slotPosition = 3;
    }
    else if(cellLimit4.get() == status){
      slotPosition = 4;
    }
    else if(cellLimit4.get() == !status){
      slotPosition = 5;
    }
    else {
      System.out.print("getSlotPosition Error");
    }

    return slotPosition;
  }

  //Getting current angle in actual degrees
  public double getMagazineAngle() {
    return (magazineSRX.getSensorCollection().getQuadraturePosition() * RobotMap.MAGAZINE_ANGLE_PER_PULSE);
  }

  public void reportDrumMagSensors() {
    SmartDashboard.putBoolean("Cell 1 Status", cellLimit1.get());
    SmartDashboard.putBoolean("Cell 2 Status", cellLimit2.get());
    SmartDashboard.putBoolean("Cell 3 Status", cellLimit3.get());
    SmartDashboard.putBoolean("Cell 4 Status", cellLimit4.get());
    SmartDashboard.putBoolean("Cell 5 Status", cellLimit5.get());
    SmartDashboard.putNumber("Amount of Power Cells", getSlotPosition());
    SmartDashboard.putNumber("Current Angle (Raw)", magazineSRX.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Current Angle", getMagazineAngle());
    SmartDashboard.putString("Shooter/Infeed Mode", getMagMode());
   
  }
}

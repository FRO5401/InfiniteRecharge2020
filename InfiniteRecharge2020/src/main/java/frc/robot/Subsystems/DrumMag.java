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
import frc.robot.Commands.DrumMagPID;

public class DrumMag extends Subsystem {

  public TalonSRX magazineSRX;
  public Solenoid cellEjectorSolenoid;

  public DigitalInput cellLimit1, cellLimit2, cellLimit3, cellLimit4, cellLimit5;

  private int loopIndex, slotIndex;
  public String magMode;

  private double magazine_kF = 0;
  private double magazine_kP = 0;
  private double magazine_kI = 0;
  private double magazine_kD = 0;
  private double magazineRotationSpeed = 0;
  private double infeedPositionPID, shooterPositionPID;

  public boolean magBoolean;
  public int currentCellPosition = 1;

  //TODO: GOES INTO ROBOTMAP
  private double nativeUnitsFor32Degrees;

  public DrumMag() {

    loopIndex = 0;
    slotIndex = 0;

    // Speed Controller for Magazine
    magazineSRX = new TalonSRX(RobotMap.MAGAZINE_TALON_CHANNEL);

    // Solenoid
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
    setDefaultCommand(new DrumMagPID());
  }

  //PID related methods, used for the Geneva Mechanism
  public void setPoint(double setPoint) {
    double setPointNativeUnits = setPoint / RobotMap.MAGAZINE_ANGLE_PER_PULSE;
    magazineSRX.set(ControlMode.Position, setPointNativeUnits);
  }

  public boolean onTarget() {
    boolean onTarget = Math.abs(magazineSRX.getSensorCollection().getQuadraturePosition()
        - magazineSRX.getClosedLoopTarget(loopIndex)) < RobotMap.MAGAZINE_THRESHOLD_FOR_PID;
    return onTarget;
  }


  //Solenoid related methods, lets you control the ejector
  public void ejectSolenoid(){
    cellEjectorSolenoid.set(true);
  }

  public void retractSolenoid(){
    cellEjectorSolenoid.set(false);
  }
  
  //Positional related methods, allows you to switch between Infeed/Shooter
  public void setMagMode(){

    if(magMode.equals("infeed")){
      //How to get to shooter
      resetCurrentCellPosition();
      magMode = "shooter";
      magBoolean = true;
    }
  
    if(magMode.equals("shooter")){
      //How to get to infeed
      resetCurrentCellPosition();
      magMode = "infeed";
      magBoolean = false;
    }
  }

  public String getMagMode(){
    return magMode;
  }

  public boolean getMagBoolean(){
    return magBoolean;
  }

  //Rotation related methods, allows Drummag to rotate a certain number of degrees
  public void rotate32Degrees(){
    double position = magazineSRX.getSensorCollection().getQuadraturePosition();
    setPoint(position + (nativeUnitsFor32Degrees));
  }

  public void rotate72Degrees(){
    double position = magazineSRX.getSensorCollection().getQuadraturePosition();
    setPoint(position + (nativeUnitsFor32Degrees * 2));
  }

  //Cell related methods, tells you which slot you are currently at
  public int getCurrentCellPosition(){
    return currentCellPosition;
  }

  public void resetCurrentCellPosition(){
    currentCellPosition = 1;
  }

  public void updateCurrentCellPosition(){
    if(currentCellPosition <= 4){
      currentCellPosition = currentCellPosition + 1;
    }
  }


  //Slot related methods, tells us which slot is being occupied due to parameters
  public boolean getLimitPressed(int limit) {

    boolean limitPressed = false;

    switch (limit) {
    case 1:
      limitPressed = cellLimit1.get();
      break;
    case 2:
      limitPressed = cellLimit2.get();
      break;
    case 3:
      limitPressed = cellLimit3.get();
      break;
    case 4:
      limitPressed = cellLimit4.get();
      break;
    case 5:
      limitPressed = cellLimit5.get();
      break;
    default:
      System.out.print("getSlotOccupied Error");
    }
    
    return limitPressed;
  }

  //Methods that report to the Driver/Operator
  public void reportDrumMagSensors() {
    SmartDashboard.putBoolean("Cell 1 Status", cellLimit1.get());
    SmartDashboard.putBoolean("Cell 2 Status", cellLimit2.get());
    SmartDashboard.putBoolean("Cell 3 Status", cellLimit3.get());
    SmartDashboard.putBoolean("Cell 4 Status", cellLimit4.get());
    SmartDashboard.putBoolean("Cell 5 Status", cellLimit5.get());
    SmartDashboard.putNumber("Amount of Power Cells", getCurrentCellPosition());
    SmartDashboard.putNumber("Current Angle (Raw)", magazineSRX.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putString("Shooter/Infeed Mode", getMagMode());
   
  }
}

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

/**
 * Add your docs here.
 */
public class DrumMag extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX magazineSRX;
  public Solenoid ballPuncher;

  int[] infeedSlots = new int[] { 0, 72, 144, 216, 288 }; // Pickup Setpoints (in degrees)
  int[] shooterSlots = new int[] { 180, 252, 324, 36, 108 }; // Shooting Setpoints (in degrees)

  DigitalInput ballLimit1, ballLimit2, ballLimit3, ballLimit4, ballLimit5;

  private boolean facingShooter;

  private int loopIndex, slotIndex;

  private double magazine_kF = 0;
  private double magazine_kP = 0;
  private double magazine_kI = 0;
  private double magazine_kD = 0;

  public DrumMag() {

    facingShooter = false;

    loopIndex = 0;
    slotIndex = 0;

    //Speed controller for the drum mag
    magazineSRX = new TalonSRX(RobotMap.MAGAZINE_TALON_CHANNEL);

    //Solenoid
    ballPuncher = new Solenoid(RobotMap.MAGAZINE_BALL_PUNCHER_CHANNEL);

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
  }

  // Sets the point to which the magazine will move
  public void setPoint(double setPoint) {
    double setPointNativeUnits = setPoint / (RobotMap.MAGAZINE_ANGLE_PER_PULSE);
    magazineSRX.set(ControlMode.Position, setPointNativeUnits);
  }


  public boolean onTarget() {
    // Method returns true if on target
    boolean onTarget = Math.abs(magazineSRX.getSensorCollection().getQuadraturePosition()
        - magazineSRX.getClosedLoopTarget(loopIndex)) < RobotMap.MAGAZINE_THRESHOLD_FOR_PID;
    return onTarget;
    // getClosedLoopT gets the desired angle
  }

    //Getting current angle in actual degrees
    public double getMagAngle() {
      return (magazineSRX.getSensorCollection().getQuadraturePosition() * (RobotMap.MAGAZINE_ANGLE_PER_PULSE));
    }
  
    //Getting current slot that is facing the infeed
     /* 
        Slot that would be facing either infeed or shooter. 
        **If on integer from 1-5, will be facing infeed
        **If one 0.5 value, will be facing shooter
      Angle/72 because total angle of circle is 360, and there are 5 slots, +1 to account for physical position.
      */
    public int getCurrentSlot() {
      int slot = (int) (getMagAngle() / 72) + 1;
      return slot;
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

  public void punchBall() {
    ballPuncher.set(true);
  }

  public void retractPuncher(){
    ballPuncher.set(false);
  }

  public void rotateToInfeed(){
    setPoint(infeedSlots[0]);
    swapMode();
  }

  public void rotateToShooter(){
    setPoint(shooterSlots[0]);
    swapMode();
  }

  public void swapMode(){ //Combined method for indicating which way the drummag is facing
    if(facingShooter == false){
      facingShooter = true;
    }
    else if(facingShooter == true){
      facingShooter = false;
    }
  }

  public boolean getMode(){
      return facingShooter;
  }

  public void reportDrumMagSensors() {
    SmartDashboard.putBoolean("Slot 1 Status", getSlotOccuppied(1));
    SmartDashboard.putBoolean("Slot 2 Status", getSlotOccuppied(2));
    SmartDashboard.putBoolean("Slot 3 Status", getSlotOccuppied(3));
    SmartDashboard.putBoolean("Slot 4 Status", getSlotOccuppied(4));
    SmartDashboard.putBoolean("Slot 5 Status", getSlotOccuppied(5));
    SmartDashboard.putNumber("Current Angle (Raw)", magazineSRX.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Current Angle", getMagAngle());
    SmartDashboard.putNumber("Current Slot", getCurrentSlot());
    SmartDashboard.putBoolean("Ready to Shoot", getMode());
   
  }
}

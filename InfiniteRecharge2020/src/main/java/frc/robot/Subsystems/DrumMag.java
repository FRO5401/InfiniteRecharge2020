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

  boolean[] ballLimitArray = new boolean[] {getSlotOccuppied(1), getSlotOccuppied(2),
    getSlotOccuppied(3), getSlotOccuppied(4), getSlotOccuppied(5)}; //Limit switch values for logic

  DigitalInput ballLimit1, ballLimit2, ballLimit3, ballLimit4, ballLimit5; //Actual limit switches

  private String currentMode;

  private int loopIndex, slotIndex;

  private double magazine_kF = 0;
  private double magazine_kP = 0;
  private double magazine_kI = 0;
  private double magazine_kD = 0;

  public DrumMag() {

    currentMode = "Infeed"; // default facing infeed

    loopIndex = 0;
    slotIndex = 0;

    // Speed Controller
    magazineSRX = new TalonSRX(RobotMap.MAGAZINE_TALON_CHANNEL);

    // Solenoid
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
        **If on integer from 1-5, will be facing infeed
        **If on a .5 value, will be facing shooter
      Angle/72 because total angle of circle is 360, and there are 5 slots, +1 to account for physical position.
      */
    public int getCurrentSlot() {
      int slot = (int) (getMagAngle() / 72) + 1;
      return slot;
    }

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
    if(currentMode == "Infeed"){
      currentMode = "Shooter";
    }
    else if(currentMode == "Shooter"){
      currentMode = "Infeed";
    }
  }

  public String getMode(){
      return currentMode;
  }

  // Logic for determining when to turn to the next slot when infeeding
    //Runs through each value of the array for limit swithces, checking if each is 'true' through each iteration.
    //Once end is reached, and all balls are in, automatically switch to shooter
  public void infeedBalls(){
    for (int x = 0; x < infeedSlots.length; x++) {
      if (withinRange(getMagAngle(), infeedSlots[x]) && ballLimitArray[x] == true) {
        if ((ballLimitArray[0] == true) && (ballLimitArray[1] == true) && (ballLimitArray[2] == true) && (ballLimitArray[3] == true) && (ballLimitArray[4] == true)){ //If all slots full, switch to shooter
          rotateToShooter();
        }
        else if ((x + 1) < 5){ //If not at last slot, keep rotating
          setPoint(infeedSlots[x + 1]);
        }
      }
      else if(ballLimitArray[x] == false) {
        x = x-1; //Prevents the loop from restarting and checking the other slots if they're already passed
      }
    }
  }

  // Logic for determining when to turn to the next slot when shooting     
    //Runs through each value of the array for limit switches, checking if each is 'false' through each iteration. 
    //Once end is reached, and all balls are out, automatically switch back to infeed.
  public void shootBalls(){
    for (int x = 0; x < shooterSlots.length; x++) { 
      if (withinRange(getMagAngle(), shooterSlots[x]) && ballLimitArray[x] == false){ //TODO: Make sure vision is targeted also 
        //Add wait command to ensure ball is out when puncher is activated(1 second for now)                          
        retractPuncher();
        //wait command to ensure retraction (try 2 seconds). Don't want turning too early.
        if ((ballLimitArray[0] == false) && (ballLimitArray[1] == false) && (ballLimitArray[2] == false) && (ballLimitArray[3] == false) && (ballLimitArray[4] == false)) { //If all slots empty, switch to infeed
          rotateToInfeed();
        }
        else if ((x + 1) < 5) { //If not at last slot, keep rotating
          setPoint(shooterSlots[x + 1]);
        }
      }   
      else if (ballLimitArray[x] == true) {
        x = x-1; //Prevents the loop from restarting and checking the other slots if they're already passed
      }                  
    }
  }

  //Checks to make sure slot is within reasonable distance from target
  private boolean withinRange(double actual, double target) {
    return Math.abs(actual - target) < (RobotMap.MAG_ERROR_TOLERANCE); //2 degrees
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
    SmartDashboard.putString("Current Mode = ", getMode());
  }
}

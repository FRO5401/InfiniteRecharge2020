/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import java.util.stream.IntStream;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Commands.DrumControl;

/**
 * Add your docs here.
 */
public class DrumMag extends Subsystem {
  private DigitalInput cell1, cell2, cell3, cell4, cell5;
  private DigitalInput kicker;
  private DigitalInput genevaLimit;
  private DigitalInput homingLimit;
  private TalonSRX genevaMotor;
  private Solenoid puncher1;
  private Solenoid puncher2;

  private int magMode;
  private boolean target; // Used in findDesiredPosition()
  private int currentPosition;
  private boolean finishedRotating; // Used to prevent currentPosition from endlessly incrementing

  boolean[] cellLimits;

  public DrumMag() {
    // PLACEHOLDERS FOR PORTS
    cell1 = new DigitalInput(0);
    cell2 = new DigitalInput(0);
    cell3 = new DigitalInput(0);
    cell4 = new DigitalInput(0);
    cell5 = new DigitalInput(0);

    puncher1 = new Solenoid(0);
    puncher2 = new Solenoid(0);

    kicker = new DigitalInput(0);

    genevaLimit = new DigitalInput(0);

    homingLimit = new DigitalInput(0);

    genevaMotor = new TalonSRX(0);

    magMode = 1; // Initializes in shooter mode
    target = true; // Target must be true for shooter mode (looking for where ball is present)
    finishedRotating = true; // Starts true

    // Will check to see if the ball is in the mag.
    cellLimits[0] = cell1.get();
    cellLimits[1] = cell1.get();
    cellLimits[2] = cell1.get();
    cellLimits[3] = cell1.get();
    cellLimits[4] = cell1.get();

  }

  // 1 is shooter, 0 is infeed. This method will switch the modes.
  public void changeMode() {
    magMode = 1 - magMode;
  }

  // This will punch or retract the solenoid depending on what is passed
  public void punchBall(boolean status) {
    puncher1.set(status);
    puncher2.set(status);
  }

  // Rotates 36 degrees (one geneva turn)
  public void rotate() {
    genevaMotor.set(ControlMode.PercentOutput, 0.1); // Slow speed for testing

    if (getGenevaLimit() && finishedRotating == false) {
      incrementPosition(); // Finishes moving, increments position
      finishedRotating = true;
    }
  }

  // Sets finishedRotating to false
  public void switchFinishedRotating() {
    finishedRotating = false;
  }

  // Returns finishedRotating
  public boolean getFinishedRotating() {
    return finishedRotating;
  }

  // Stops drummag motor
  public void stop() {
    genevaMotor.set(ControlMode.PercentOutput, 0.0);
    finishedRotating = true;
  }

  // Finds position to turn to next
  public int findDesiredPosition() {
    int desiredPosition;
    getCellLimits();
    if (magMode == 1) {
      target = true;
    } else if (magMode == 0) {
      target = false;
    }
    // This SHOULD equal the first index in the cell limit array that equals the
    // variable target
    int firstValueIndex = IntStream.range(0, cellLimits.length).filter(i -> target == cellLimits[i]).findFirst()
        .orElse(-1);
    // This math makes desiredPosition equal the next position the mag needs to go
    // to
    desiredPosition = ((2 * firstValueIndex) + (1 - magMode) * 5) % 10;
    return desiredPosition;
  }

  // Gets power cell status
  public boolean[] getCellLimits() {
    cellLimits[0] = cell1.get();
    cellLimits[1] = cell2.get();
    cellLimits[2] = cell3.get();
    cellLimits[3] = cell4.get();
    cellLimits[4] = cell5.get();
    return cellLimits;
  }

  // Gets position for use in commands
  public int getPosition() {
    return currentPosition;
  }

  // Increments position by 1
  /*
   * Shooter 1 = 0 Infeed 4 = 1 Shooter 2 = 2 Infeed 5 = 3 Shooter 3 = 4 Infeed 1
   * = 5 Shooter 4 = 6 Infeed 2 = 7 Shooter 5 = 8 Infeed 3 = 9
   */
  public void incrementPosition() {
    if (currentPosition == 9) {
      currentPosition = 0;
    } else {
      currentPosition += 1;
    }
  }

  // Gets limit status
  public boolean getKickerLimit() {
    return kicker.get();
  }

  // Gets limit status
  public boolean getGenevaLimit() {
    return genevaLimit.get();
  }

  // Gets limit status
  public boolean getHomingLimit() {
    return homingLimit.get();
  }

  // Resets position to 0 (called when homing limit is true)
  public void resetPosition() {
    currentPosition = 0;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DrumControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}

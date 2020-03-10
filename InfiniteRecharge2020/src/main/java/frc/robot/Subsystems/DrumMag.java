/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Commands.DrumControl;

/**
 * Add your docs here.
 */
public class DrumMag extends Subsystem {
//  private DigitalInput cell1, cell2, cell3, cell4, cell5;
  private VictorSPX magazineMotor;
  private VictorSPX kickerMotor;

  public DrumMag() {

    magazineMotor = new VictorSPX(RobotMap.MAGAZINE_MOTOR_CHANNEL);
    kickerMotor = new VictorSPX(RobotMap.KICKER_MOTOR_CHANNEL);

  }

  // This will raise balls to the shooter
  public void loadBall() {
    kickerMotor.set(ControlMode.PercentOutput, RobotMap.KICKER_SPEED);
  }

  public void stopLoading(){
    kickerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  // Rotates 36 degrees (one geneva turn)
  public void rotateMagazine() {
    magazineMotor.set(ControlMode.PercentOutput, RobotMap.MAGAZINE_SPEED);
  }

  //Stops magazine motor
  public void stopMagazine() {
    magazineMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void reportValues(){
    //TODO: Add reports here
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DrumControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
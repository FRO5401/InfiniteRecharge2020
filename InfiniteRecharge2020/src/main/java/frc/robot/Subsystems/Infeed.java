/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Commands.InfeedControl;

/**
 * Add your docs here.
 */
public class Infeed extends Subsystem {
  //2 motors, 4 solenoid
  //MIGHT NOT BE VICTOR SP, check with design
  private VictorSP infeedMotor1;
  private VictorSP infeedMotor2;
  private Solenoid deployInfeed;

  public Infeed(){
    infeedMotor1 = new VictorSP(0);
    infeedMotor2 = new VictorSP(0);
    deployInfeed = new Solenoid(0);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new InfeedControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //Deploy Infeed
  public void deployInfeed(boolean status){
    deployInfeed.set(status);
  }

  public boolean getDeployStatus(){
    boolean status = deployInfeed.get();
    return status;
  }

  public void runInfeed(String direction){
    if(direction.equals("IN")){
      infeedMotor1.set(RobotMap.INFEED_SPEED);
      infeedMotor2.set(RobotMap.INFEED_SPEED);
    }
    else if(direction.equals("OUT")){
      infeedMotor1.set(-1 * RobotMap.INFEED_SPEED);
      infeedMotor2.set(-1 * RobotMap.INFEED_SPEED);
    }
    else if(direction.equals("STOP")){
      infeedMotor1.set(0);
      infeedMotor2.set(0);
    }
    else{
      System.out.print("BRUH");
    }
  }

}

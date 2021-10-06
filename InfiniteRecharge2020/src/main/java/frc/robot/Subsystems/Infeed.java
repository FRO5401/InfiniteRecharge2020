package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Infeed extends SubsystemBase {
  private WPI_VictorSPX infeedMotor1;
  private WPI_VictorSPX infeedMotor2;
  private Solenoid deployInfeed;

  public Infeed(){
    infeedMotor1 = new WPI_VictorSPX(RobotMap.INFEED_MOTOR_1);
    infeedMotor2 = new WPI_VictorSPX(RobotMap.INFEED_MOTOR_2);
    deployInfeed = new Solenoid(RobotMap.INFEED_DEPLOY);
  }

  @Override
  public void periodic() {
    reportValues();      
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

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
      infeedMotor1.set(ControlMode.PercentOutput, RobotMap.INFEED_SPEED);
      infeedMotor2.set(ControlMode.PercentOutput, RobotMap.INFEED_SPEED);
    }
    else if(direction.equals("OUT")){
      infeedMotor1.set(ControlMode.PercentOutput, -1 * RobotMap.INFEED_SPEED);
      infeedMotor2.set(ControlMode.PercentOutput, -1 * RobotMap.INFEED_SPEED);
    }
    else if(direction.equals("STOP")){
      infeedMotor1.set(ControlMode.PercentOutput, 0);
      infeedMotor2.set(ControlMode.PercentOutput, 0);
    }
    else{
      System.out.print("BRUH");
    }
  }

  public void reportValues(){
    SmartDashboard.putBoolean("Infeed Status", getDeployStatus());
  }
}

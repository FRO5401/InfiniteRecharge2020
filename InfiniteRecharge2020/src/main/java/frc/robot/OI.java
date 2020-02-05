/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
/**
 * Add your docs here.
 */
public class OI extends Subsystem {
  
  Joystick xboxController_Driver = new Joystick(RobotMap.XBOX_CONTROLLER_DRIVER);
  Joystick XboxController_Operator = new Joystick(RobotMap.XBOX_CONTROLLER_DRIVER);

  ///Buttons 

  //Driver

  Button xboxA_Driver = new JoystickButton(xboxController_Driver, 1);
  Button xboxY_Driver = new JoystickButton(xboxController_Driver, 2);
  Button leftBumper_Driver = new JoystickButton(xboxController_Driver, 3);
  Button rightBumper_Driver = new JoystickButton(xboxController_Driver, 4);
  Button start_Driver = new JoystickButton(xboxController_Driver, 5);
  Button back_Driver = new JoystickButton(xboxController_Driver, 6);
  Button leftStick_Driver = new JoystickButton(xboxController_Driver, 7);
  Button rightStick_Driver = new JoystickButton(xboxController_Driver, 8);

  //Operator

  Button xboxA_Operator = new JoystickButton(XboxController_Operator, 1);
  Button xboxB_Operator = new JoystickButton(XboxController_Operator, 2);
  Button xboxX_Operator = new JoystickButton(XboxController_Operator, 3);
  Button xboxY_Operator = new JoystickButton(XboxController_Operator, 4);
  Button leftBumper_Operator = new JoystickButton(XboxController_Operator, 5);
  Button rightBumper_Operator = new JoystickButton(XboxController_Operator, 6);
  Button rightStick_Operator= new JoystickButton(XboxController_Operator, 7);
  Button rightTrigger_Operator= new JoystickButton(XboxController_Operator, 8);
  Button leftTrigger_Operator= new JoystickButton(XboxController_Operator, 9);

  
  public OI() {

    //Flip Button
    xboxY_Driver.whenPressed(new CompressorToggle());

  


  }

  

  





  

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

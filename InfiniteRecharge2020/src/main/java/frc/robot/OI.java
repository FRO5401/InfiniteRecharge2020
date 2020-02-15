/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*------frc.robot----------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
 
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
 
 
 
public class OI{
  public Joystick xboxDriver = new Joystick(RobotMap.XBOX_CONTROLLER_DRIVER);
  public Joystick xboxOperator = new Joystick(RobotMap.XBOX_CONTROLLER_OPERATOR);
 
    //Buttons (Driver)
    Button xboxA_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_A);
    Button xboxB_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_B);
    Button xboxX_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_X);
    Button xboxY_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_Y);
    Button xboxLeftBumper_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    Button xboxRightBumper_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
    Button xboxBack_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_BACK);
    Button xboxStart_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_START);
    Button xboxL3_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_LS);
    Button xboxR3_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_RS);
  
    //Buttons (Operator)
    Button xboxA_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_A);
    Button xboxB_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_B);
    Button xboxX_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_X);
    Button xboxY_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_Y);
    Button xboxLeftBumper_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    Button xboxRightBumper_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
    Button xboxBack_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_BACK);
    Button xboxStart_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_START);
    Button xboxL3_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_LS);
    Button xboxR3_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_RS);

    
    public double xboxAxis(Joystick controller, int xboxAxis) {
      return controller.getRawAxis(xboxAxis);
      
    }
    public boolean xboxButton(Joystick xboxController, int xboxButton){
      return xboxController.getRawButton(xboxButton);
    }

}




package frc.robot;



import frc.robot.RobotMap;
import frc.robot.Commands.CompressorToggle;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;



public class OI{
  public Joystick xboxDriver = new Joystick(RobotMap.XBOX_CONTROLLER_DRIVER);
  public Joystick xboxOperator = new Joystick(RobotMap.XBOX_CONTROLLER_OPERATOR);

    //Buttons (Driver)
  Button xboxA_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_A);
	Button xboxB_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_B);
	Button xboxX_Driver	= new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_X);
	Button xboxY_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_Y);
	Button xboxLeftBumper_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
	Button xboxRightBumper_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
	Button xboxBack_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_BACK);
	Button xboxStart_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_START);
	Button xboxL3_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_L3);
	Button xboxR3_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_R3);
  
    //Buttons (Operator)
  Button xboxA_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_A);
	Button xboxB_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_B);
	Button xboxX_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_X);
	Button xboxY_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_Y);
	Button xboxLeftBumper_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
	Button xboxRightBumper_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
	Button xboxBack_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_BACK);
	Button xboxStart_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_START);
	Button xboxL3_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_L3);
  Button xboxR3_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_R3);
  


  public OI() {
	
	xboxY_Driver.whenPressed(new CompressorToggle());

    


  }
}

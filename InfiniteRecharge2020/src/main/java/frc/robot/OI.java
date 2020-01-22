package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	Joystick xboxController_Driver = new Joystick(RobotMap.XBOX_CONTROLLER_DRIVER);
	Joystick xboxController_Operator = new Joystick(RobotMap.XBOX_CONTROLLER_OPERATOR);

	Button xboxLeftBumper_Driver  = new JoystickButton(xboxController_Driver, 5);
    Button xboxRightBumper_Driver = new JoystickButton(xboxController_Driver, 6);
    
	Button xboxLeftBumper_Operator  = new JoystickButton(xboxController_Operator, 5);
    Button xboxRightBumper_Operator = new JoystickButton(xboxController_Operator, 6);
    
    public int getXboxTriggers_Operator(){
		double left  = xboxController_Operator.getRawAxis(RobotMap.XBOX_AXIS_LEFT_TRIGGER);
		double right = xboxController_Operator.getRawAxis(RobotMap.XBOX_AXIS_RIGHT_TRIGGER);
		if (right > .1){ 
			return 1;
		} else if (left > .1){//<--left is in
			return -1;
		} else {
			return 0;
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import frc.robot.Commands.*;
/**
 * Add your docs here.
 */
public class OI {
    //Reee


    //Controllers (Joysticks)
    public Joystick xboxDriver = new Joystick(RobotMap.XBOX_CONTROLLER_DRIVER);
    public Joystick xboxOperator = new Joystick(RobotMap.XBOX_CONTROLLER_OPERATOR);
    
    //Driver Countroller Buttons

    Button xboxA_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_A);
    Button xboxB_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_B);
    Button xboxX_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_X);
    Button xboxY_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_Y);
    Button xboxLB_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_LB);
    Button xboxRB_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_RB);
    Button xboxStart_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_START);
    Button xboxBack_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_BACK);
    Button xboxL3_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_L3);
    Button xboxR3_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_R3);

    //Operator Controller Buttons
    Button xboxA_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_A);
    Button xboxB_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_B);
    Button xboxX_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_X);
    Button xboxY_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_Y);
    Button xboxLB_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_LB);
    Button xboxRB_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_RB);
    Button xboxStart_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_START);
    Button xboxBack_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_BACK);
    Button xboxL3_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_L3);
    Button xboxR3_Driver = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_R3);

    public OI(){
        //Toggle Compressor
      xboxY_Driver.whenPressed(new CompressorToggle());
    }

    public double xboxAxis(Joystick xboxController, int xboxAxis){
        return xboxController.getRawAxis(xboxAxis);
    }

    public boolean xboxButton(Joystick xboxController, int xboxButton){
        return xboxController.getRawButton(xboxButton);
    }

    public int xboxAxisAsButton(Joystick xboxController, int xboxAxis){
        if(xboxController.getRawAxis(xboxAxis) > RobotMap.AXIS_THRESHOLD){
            return 1;
            }
        else if(xboxController.getRawAxis(xboxAxis) < (-1 * RobotMap.AXIS_THRESHOLD)){
            return -1;
        }
        else{
            return 0;
          }
    }

    public int xboxDPad(Joystick xboxController){
        return xboxController.getPOV();
      }
}

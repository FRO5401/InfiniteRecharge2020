/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  /*** Constants ***/
  // OI
  public static final double AXIS_THRESHOLD = 0.1;
  
  // DriveBase
  public static final double LOW_GEAR_LEFT_DPP = 0.1568175312974026;
  public static final double LOW_GEAR_RIGHT_DPP = 0.1568175312974026;
  public static final double HIGH_GEAR_LEFT_DPP = 0;
  public static final double HIGH_GEAR_RIGHT_DPP = 0;

  // XboxMove
  public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
  public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
  public static final double SPIN_SENSITIVITY = 0.8;

  // Magazine
  public static final int TIMEOUT_LIMIT_IN_Ms = 10;
  public static final double MAGAZINE_ANGLE_PER_PULSE = 0;
  public static final int MAGAZINE_THRESHOLD_FOR_PID = 0;
  public static final double MAGAZINE_SPEED_SENSITIVITY = .5;

  // Limits
  public static final int MAGAZINE_STOP_1 = 0;
  public static final int MAGAZINE_STOP_2 = 0;
  public static final int MAGAZINE_STOP_3 = 0;
  public static final int MAGAZINE_STOP_4 = 0;
  public static final int MAGAZINE_STOP_5 = 0;

  public static final int EJECTOR_RETRACTED = 0;

  /*** Operator Interfaces ***/
  // Controllers
  public static final int XBOX_CONTROLLER_DRIVER = 0;
  public static final int XBOX_CONTROLLER_OPERATOR = 1;

  // Buttons
  public static final int XBOX_BUTTON_A = 1;
  public static final int XBOX_BUTTON_B = 2;
  public static final int XBOX_BUTTON_X = 3;
  public static final int XBOX_BUTTON_Y = 4;
  public static final int XBOX_BUTTON_LEFT_BUMPER = 5;
  public static final int XBOX_BUTTON_RIGHT_BUMPER = 6;
  public static final int XBOX_BUTTON_BACK = 7;
  public static final int XBOX_BUTTON_START = 8;
  public static final int XBOX_BUTTON_L3 = 9;
  public static final int XBOX_BUTTON_R3 = 10;

  // Axes
  public static final int XBOX_AXIS_LEFT_X = 0;
  public static final int XBOX_AXIS_LEFT_Y = 1;
  public static final int XBOX_AXIS_LEFT_TRIGGER = 2;
  public static final int XBOX_AXIS_RIGHT_TRIGGER = 3;
  public static final int XBOX_AXIS_RIGHT_X = 4;
  public static final int XBOX_AXIS_RIGHT_Y = 5;

  /*** Motors ***/
  // DriveBase
  public static final int DRIVE_MOTOR_RIGHT_1 = 0;
  public static final int DRIVE_MOTOR_LEFT_1 = 2;
  public static final int DRIVE_MOTOR_RIGHT_2 = 1;
  public static final int DRIVE_MOTOR_LEFT_2 = 3;

  // CarriageInfeed
  public static final int CARRIAGE_FEED_ROLLERS = 4;
  public static final int CARRIAGE_TALON_CHANNEL = 2;
  public static final int C_STOP_T = 7;

  // Magazine
  public static final int MAGAZINE_TALON_CHANNEL = 0;

  /*** Solenoids (Single and Double) ***/
  // DoubleSolenoids have an IN and an OUT constant.
  // Solenoids have just one constant.
  // PCM (Pneumatic Control Module)
  public static final int PCM_ID = 0;

  // DriveBase
  public static final int GEAR_SHIFTER = 0;

  // Elevator
  public static final int ELEVATOR_GEAR_SHIFTER = 4;
  public static final int ELEVATOR_COLLAPSE_TOP = 5;
  public static final int ELEVATOR_COLLAPSE_BOTTOM = 1;

  // Power Cell Ejection
  public static final int MAGAZINE_CELL_EJECTOR_CHANNEL = 2;

  /*** Sensors ***/
  // Encoders
  public static final int DRIVE_ENC_LEFT_A = 3;
  public static final int DRIVE_ENC_RIGHT_A = 1;
  public static final int DRIVE_ENC_LEFT_B = 4;
  public static final int DRIVE_ENC_RIGHT_B = 2;

  // Elevator
  public static final int E_STOP_HIGH = 5;
  public static final int E_STOP_LOW = 6;

  // DOUBLE SOLENOIDS ELEVATOR
  public static final int BOTTOM_ELEVATOR_OUT = 6;
  public static final int BOTTOM_EVEVATOR_IN = 7;

  /*** Autonomous ***/
  public static final int ANGLE_THRESHOLD = 2;
  public static final double AUTO_TURN_SPEED = 0.8;
  public static final double AUTO_TURN_PRECISION = 0.5;


  public static final Boolean PLACE_HOLDER = null;
}
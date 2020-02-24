/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
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
  public static final double AXIS_THRESHOLD = 0.25;

  public static final double SPEED_ADJUSTMENT_LEFT_FORWARD = 1.09;
  public static final double SPEED_ADJUSTMENT_LEFT_BACKWARD = 1.09;
  public static final double LOW_GEAR_LEFT_DPP = 0.000570664409648; // TODO: Make smaller until skew is gone
  public static final double LOW_GEAR_RIGHT_DPP = 0.000618865317636; // Low gear dpp 2020
  public static final double LOW_GEAR_AVERAGE_DPP = 0.000712464466462;
  public static final double HIGH_GEAR_LEFT_DPP = 0.000650533437419; // High gear skews left
  public static final double HIGH_GEAR_RIGHT_DPP = 0.000666062769755; // High gear dpp 2020

  // XboxMove
  public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
  public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
  public static final double SPIN_SENSITIVITY = 0.8;

  // Turret
  public static final double TURRET_TURN_SPEED = .35;
  public static final double TURRET_ANGLE_PER_PULSE = 5511.5889;

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
  public static final int DRIVE_MOTOR_RIGHT_1 = 1;
  public static final int DRIVE_MOTOR_RIGHT_2 = 3;
  public static final int DRIVE_MOTOR_RIGHT_3 = 5;

  public static final int DRIVE_MOTOR_LEFT_1 = 2;
  public static final int DRIVE_MOTOR_LEFT_2 = 4;
  public static final int DRIVE_MOTOR_LEFT_3 = 6;

  // Turret
  public static final int TURRET_TALON = 10;
  public static final int T_STOP_LEFT = 90;
  public static final int T_STOP_RIGHT = -90;
  public static final double TURRET_SPEED_SENSITIVITY = 0.35;
  public static final double TURRET_ANGLE_LIMIT = 90;

  // Shooter
  public static final int SHOOTER_MASTER_CHANNEL = 7;
  public static final int SHOOTER_SLAVE_CHANNEL = 8;

  /*** Solenoids (Single and Double) ***/
  // DoubleSolenoids have an IN and an OUT constant.
  // Solenoids have just one constant.
  // PCM (Pneumatic Control Module)
  public static final int PCM_ID = 0;

  // DriveBase
  public static final int GEAR_SHIFTER = 0;

  /*** Sensors ***/
  // Encoders
  public static final int DRIVE_ENC_LEFT_A = 3;
  public static final int DRIVE_ENC_RIGHT_A = 1;
  public static final int DRIVE_ENC_LEFT_B = 4;
  public static final int DRIVE_ENC_RIGHT_B = 2;

  /*** Autonomous ***/
  public static final int ANGLE_THRESHOLD = 2;
  public static final double AUTO_TURN_SPEED = 0.8;
  public static final double AUTO_TURN_PRECISION = 0.5;
}
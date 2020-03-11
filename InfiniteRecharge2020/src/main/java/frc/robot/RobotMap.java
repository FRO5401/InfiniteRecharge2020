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
  public static final double AXIS_THRESHOLD = 0.25;
  public static final int ELEVATOR_BUTTON_SHIFT_HIGH = 1;
  public static final int ELEVATOR_BUTTON_SHIFT_LOW = 2;
  public static final int ELEVATOR_BUTTON_COLLAPSE = 3;
  public static final int ELEVATOR_BUTTON_RISE = 4;

  // DriveBase
  public static final double TELEOP_SPEED_ADJUSTMENT_LEFT = 0.94; //TODO: Update for 2020 Comp Bot
  public static final double LOW_GEAR_LEFT_DPP = 0.1466004558282468;
  public static final double LOW_GEAR_RIGHT_DPP  = 0.1568175312974026;
  public static final double HIGH_GEAR_LEFT_DPP = 0.1568175312974026;
  public static final double HIGH_GEAR_RIGHT_DPP = 0.1466004558282468;

  //Infeed
  public static final double INFEED_SPEED = 0.55;

  //Feeder
  public static final double FEEDER_SPEED = 0.75;

  // XboxMove
  public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
  public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
  public static final double SPIN_SENSITIVITY = 0.8;

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
  public static final int DRIVE_MOTOR_RIGHT_1 = 7;
  public static final int DRIVE_MOTOR_RIGHT_2 = 9;
  public static final int DRIVE_MOTOR_RIGHT_3 = 11;

  public static final int DRIVE_MOTOR_LEFT_1 = 1;
  public static final int DRIVE_MOTOR_LEFT_2 = 3;
  public static final int DRIVE_MOTOR_LEFT_3 = 5;

  //Infeed
  public static final int INFEED_MOTOR_LEFT = 13;
  public static final int INFEED_MOTOR_RIGHT = 15;

  //BeltChannel & Feeder
  public static final int BELT_MOTOR = 0;
  public static final int FEEDER_MOTOR_1 = 0;
  public static final int FEEDER_MOTOR_2 = 0;

  //Shooter
  public static final int SHOOTER_MASTER_CHANNEL = 19;
  public static final int SHOOTER_SLAVE_CHANNEL = 21;

  //Climber
  public static final int CLIMB_MOTOR_1 = 27;
  public static final int CLIMB_MOTOR_2 = 29;
  
  /*** Solenoids (Single and Double) ***/
  // DoubleSolenoids have an IN and an OUT constant.
  // Solenoids have just one constant.
  // PCM (Pneumatic Control Module)
  public static final int PCM_ID = 0;

  //DriveBase
  public static final int GEAR_SHIFTER = 0;
  
  //Infeed
  public static final int INFEED_DEPLOY_1 = 0;
  public static final int INFEED_DEPLOY_2 = 0;

  /*** Sensors ***/
  //Encoders
  public static final int DRIVE_ENC_LEFT_A = 3; //Encoders can be read through TalonSRX, VictorSPX, and TalonFX. These might not be needed.
  public static final int DRIVE_ENC_RIGHT_A = 1;
  public static final int DRIVE_ENC_LEFT_B = 4;
  public static final int DRIVE_ENC_RIGHT_B = 2;

  /*** Autonomous ***/
  public static final double ANGLE_THRESHOLD = 6;
  public static final double AUTO_SPEED_ADJUSTMENT = 1.4;
  public static final double AUTO_TURN_SPEED = 0.8;
  public static final double AUTO_TURN_PRECISION = 0.5;

}
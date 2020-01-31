/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


public class RobotMap {

    /**** Controller Shit ****/
    //Controllers
    public static final int XBOX_CONTROLLER_DRIVER = 0;
    public static final int XBOX_CONTROLLER_OPERATOR = 1;
    
    //Buttons
    public static final int XBOX_BUTTON_A = 1;
    public static final int XBOX_BUTTON_B = 2;
    public static final int XBOX_BUTTON_X = 3;
    public static final int XBOX_BUTTON_Y = 4;
    public static final int XBOX_BUTTON_LB = 5;
    public static final int XBOX_BUTTON_RB = 6;
    public static final int XBOX_BUTTON_START = 7;
    public static final int XBOX_BUTTON_BACK = 8;
    public static final int XBOX_BUTTON_L3 = 9;
    public static final int XBOX_BUTTON_R3 = 10;

    //Axes
    public static final int XBOX_LEFT_X_AXIS = 0;
    public static final int XBOX_LEFT_Y_AXIS = 1;
    public static final int XBOX_LT = 2;
    public static final int XBOX_RT = 3;
    public static final int XBOX_RIGHT_X_AXIS = 4;
    public static final int XBOX_RIGHT_Y_AXIS = 5;
    
    /**** Constant Variables ****/
    // OI
    public static final double AXIS_THRESHOLD = 0.25;
    public static final int ELEVATOR_BUTTON_SHIFT_HIGH = 1;
    public static final int ELEVATOR_BUTTON_SHIFT_LOW = 2;
    public static final int ELEVATOR_BUTTON_COLLAPSE = 3;
    public static final int ELEVATOR_BUTTON_RISE = 4;

    // DriveBase
    public static final double LOW_GEAR_LEFT_DPP = 0.1466004558282468; // These are for practice bot, skewed a lot
    public static final double LOW_GEAR_RIGHT_DPP = 0.1568175312974026;
    public static final double HIGH_GEAR_LEFT_DPP = 0;
    public static final double HIGH_GEAR_RIGHT_DPP = 0;

    // XboxMove
    public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
    public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
    public static final double SPIN_SENSITIVITY = 0.8;

    // CarriageInfeed
    public static final double CARRIAGE_FEEDER_SPEED = .4;
    public static final double CARRIAGE_ANGLE_PER_PULSE = 0;

    // Elevator
    public static final int TIMEOUT_LIMIT_IN_Ms = 10;
    public static final int ELEVATOR_THRESHOLD_FOR_PID = 0;
    public static final double ELEVATOR_SPEED_SENSITIVITY = .4;

    /**** Motors ****/
    //DriveBase
    public static final int DRIVE_MOTOR_R1 = 3;
    public static final int DRIVE_MOTOR_R2 = 5;
    public static final int DRIVE_MOTOR_R3 = 1;
    public static final int DRIVE_MOTOR_L1 = 4;
    public static final int DRIVE_MOTOR_L2 = 6;
    public static final int DRIVE_MOTOR_L3 = 2;

    // CarriageInfeed
    public static final int CARRIAGE_FEED_ROLLERS = 4;
    public static final int CARRIAGE_TALON_CHANNEL = 2;
    public static final int C_STOP_T = 7;

    // Elevator
    public static final int ELEVATOR_TALON_MASTER_CHANNEL = 0;
    public static final int ELEVATOR_TALON_SLAVE_CHANNEL = 1;

    /**** Solenoids (Single and Double) ****/
    // PCM (Pneumatic Control Module)
    public static final int PCM_ID = 0;

    // DriveBase
    public static final int GEAR_SHIFTER = 0;

    // Elevator
    public static final int ELEVATOR_GEAR_SHIFTER = 4;
    public static final int ELEVATOR_COLLAPSE_TOP = 5;
    public static final int ELEVATOR_COLLAPSE_BOTTOM = 1;

    // Hatch Mechanism
    public static final int HATCH_EXTENDER = 2;

    /**** Sensors ****/
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

    /**** Autonomous ****/
    public static final int ANGLE_THRESHOLD = 2;
    public static final double AUTO_TURN_SPEED = 0.8;
    public static final double AUTO_TURN_PRECISION = 0.5;

}

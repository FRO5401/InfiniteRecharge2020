/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Commands.XboxMove;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

/**
 * Add your docs here.
 */
public class KahanDriveBase extends Subsystem {
    
    //Motors
    public VictorSPX leftDrive1;
    public VictorSPX rightDrive1;
    public VictorSPX leftDrive2;
    public VictorSPX rightDrive2;
    public TalonSRX leftDrive3;
    public TalonSRX rightDrive3;

    //Gear Shift 
    public Solenoid gearShift;

    //Sensor & Gyroscope
    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public AHRS navxGyro;

    //Extanciations 
    public KahanDriveBase() {

        //Start Motor
        leftDrive1 = new VictorSPX(0);
        rightDrive1 = new VictorSPX(0);
        leftDrive2 = new VictorSPX(0);
        rightDrive2 = new VictorSPX(0);
        leftDrive3 = new TalonSRX(0);
        rightDrive3 = new TalonSRX(0);

        //Start Gear Shift
        gearShift = new Solenoid(0);

        //Start Sensor & Gyroscope
        leftEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, true, EncodingType.k4X); //Channel needs to go inside the parameters
        rightEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, false, EncodingType.k4X); //Channel needs to go inside the parameters
        navxGyro = new AHRS(I2C.Port.kMXP);
    }

    //Default Command 
    @Override 
    public void initDefaultCommand() {
        setDefaultCommand(new XboxMove());
    }

    //Motor Rotation Assign 
    public void drive(double leftAssign, double rightAssign) {
        leftDrive1.set(ControlMode.PercentOutput, leftAssign);
        leftDrive2.set(ControlMode.PercentOutput, leftAssign);
        leftDrive3.set(ControlMode.PercentOutput, leftAssign);
        rightDrive1.set(ControlMode.PercentOutput, -1 * rightAssign);
        rightDrive2.set(ControlMode.PercentOutput, -1 * rightAssign);
        rightDrive3.set(ControlMode.PercentOutput, -1 * rightAssign);
    }

    //Motor Stop
    public void stop() {
        leftDrive1.set(ControlMode.PercentOutput, 0);
        leftDrive2.set(ControlMode.PercentOutput, 0);
        leftDrive3.set(ControlMode.PercentOutput, 0);
        rightDrive1.set(ControlMode.PercentOutput, 0);
        rightDrive2.set(ControlMode.PercentOutput, 0);
        rightDrive3.set(ControlMode.PercentOutput, 0);
    }

    //Set Shifter to Low
    public void shiftHightoLow() {
        gearShift.set(true);
        setDPPLowGear();
    }

    //Set Shifter to High
    public void shiftLowtoHigh() {
        gearShift.set(true);
        setDPPHighGear();
    }

    //Set Distance Per Pulse Low
    public void setDPPLowGear() {
      leftEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_LEFT_DPP); 
      rightEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_RIGHT_DPP); 
    }

    //Set Distance Per Pulse High
    public void setDPPHighGear() {
        leftEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_LEFT_DPP);  
        rightEncoder.setDistancePerPulse((RobotMap.HIGH_GEAR_RIGHT_DPP));
    }

}

 
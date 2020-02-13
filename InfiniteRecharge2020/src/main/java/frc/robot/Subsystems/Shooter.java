/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Commands.ShooterMechanism;
import frc.robot.Commands.XboxMove;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Shooter extends Subsystem {

    // instanciate the objects
    TalonFX shooterMaster, shooterSlave;
    private double kF, kP, kI, kD;
    private double PID_MOTOR_SPEED = 0;
    private double MOTOR_SPEED = PID_MOTOR_SPEED;

    // make a constructor and declare the variables

    public Shooter() {

        // Placeholder
        TalonFX shooterMaster = new TalonFX(0); //TODO: Set to correct CAN IDs
        TalonFX shooterSlave = new TalonFX(0);

        // Sets the speed
        shooterMaster.set(ControlMode.Velocity, 0);
        shooterSlave.set(ControlMode.Follower, shooterMaster.getDeviceID());

        // Positioning
        shooterMaster.getSensorCollection().getIntegratedSensorPosition();
        shooterSlave.getSensorCollection().getIntegratedSensorPosition();

        kF = 0; //TODO: Set PID values (bruh moment)
        kP = 0;
        kI = 0;
        kD = 0;

        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        stop();

    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ShooterMechanism());

    }

    public void stop() {
        shooterMaster.set(ControlMode.Velocity, 0);
    }

    public double getTargetSpeed() {
        return MOTOR_SPEED;
    }

    public void startMotors() {
        shooterMaster.config_kF(0, kF, 1000);
        shooterMaster.config_kP(0, kP, 1000);
        shooterMaster.config_kI(0, kI, 1000);
        shooterMaster.config_kD(0, kD, 1000);
    }

    public void runMotors() {
        shooterMaster.set(ControlMode.Velocity, 100);
    }

    public double getVelocity() {
        return shooterMaster.getSensorCollection().getIntegratedSensorPosition();
    }

}

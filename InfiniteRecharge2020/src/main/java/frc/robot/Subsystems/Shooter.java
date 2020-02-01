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
import frc.robot.Commands.XboxMove;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Shooter extends Subsystem {

    // instanciate the objects
    TalonSRX shooterMaster, shooterSlave;
    private double kP, kI, kD;
    private boolean pidEnabled;
    private double PID_MOTOR_SPEED = 0;
    private double MOTOR_SPEED = PID_MOTOR_SPEED;

    private Solenoid puncher;

    // make a constructor and declare the variables

    public Shooter() {

        TalonSRX shooterMaster = new TalonSRX(0);
        TalonSRX shooterSlave = new TalonSRX(0);

        shooterMaster.set(ControlMode.Velocity, 0);
        shooterSlave.set(ControlMode.Follower, shooterMaster.getDeviceID());

        shooterMaster.getSensorCollection().getQuadraturePosition();
        shooterSlave.getSensorCollection().getQuadraturePosition();

        kP = 0;
        kI = 0;
        kD = 0;

        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        reset();

    }

    @Override
    public void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    public void reset() {
        stop();
    }

    public void stop() {
        shooterMaster.set(ControlMode.Velocity, 0);
    }

    public double getTargetSpeed() {
        return MOTOR_SPEED;
    }

    public void increaseMotorSpeed() {
        MOTOR_SPEED++;
    }

    public void decreaseMotorSpeed() {
        MOTOR_SPEED--;
    }

    public void runMotors() {
        shooterMaster.set(ControlMode.Velocity, MOTOR_SPEED);
    }

    public void startMotors() {

        if (pidEnabled) {
            shooterMaster.config_kP(0, kP, 0);
            shooterMaster.config_kI(0, kI, 0);
            shooterMaster.config_kD(0, kD, 0);
        }
    }

    public double getVelocity() {
        return shooterMaster.getSensorCollection().getQuadraturePosition();
    }

    public void reportShooter() {
        SmartDashboard.putNumber("Desired Speed", getTargetSpeed());
        SmartDashboard.putNumber("Current Speed", getVelocity());

    }

}

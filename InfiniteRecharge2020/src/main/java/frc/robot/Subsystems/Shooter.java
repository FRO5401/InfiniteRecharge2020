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

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class Shooter extends Subsystem {

    // instanciate the objects
    TalonFX shooterMaster, shooterSlave;
    private double kF, kP, kI, kD;
    private double PID_MOTOR_SPEED = 0;
    private double MOTOR_SPEED = PID_MOTOR_SPEED;

    // make a constructor and declare the variables

    public Shooter() {

        // Placeholder
        shooterMaster = new TalonFX(RobotMap.SHOOTER_MASTER_CHANNEL); //TODO: Set to correct CAN IDs
        shooterSlave = new TalonFX(RobotMap.SHOOTER_SLAVE_CHANNEL);

        //shooterSlave must follow shooterMaster inverted
        shooterSlave.follow(shooterMaster);
        shooterSlave.setInverted(true);

        SmartDashboard.putNumber("Input kF", 0.0);
        SmartDashboard.putNumber("Input kP", 0.0);
        SmartDashboard.putNumber("Input kI", 0.0);
        SmartDashboard.putNumber("Input kD", 0.0);

        SmartDashboard.putNumber("CLICK BOX FOR CLICKING", 5401);

        kF = 0;
        kP = 1;
        kI = 0;
        kD = 0;


        // Positioning
        //shooterMaster.getSensorCollection().getIntegratedSensorPosition();
        //shooterSlave.getSensorCollection().getIntegratedSensorPosition();

        //shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ShooterMechanism());

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
        shooterMaster.set(TalonFXControlMode.PercentOutput, -1*.80); //Change back to velocity after testing master/slave
        //Make first velocity 1
    }

    public void stopMotors() {
        shooterMaster.set(TalonFXControlMode.PercentOutput, 0.0); //Change back to velocity after testing master/slave
    }

    public double getVelocity() {
        return shooterMaster.getSensorCollection().getIntegratedSensorVelocity();
    }

    public void reportValues(){
        SmartDashboard.putNumber("Shooter Speed", getVelocity());
        SmartDashboard.putNumber("Current kF", kF);
        SmartDashboard.putNumber("Current kP", kP);
        SmartDashboard.putNumber("Current kI", kI);
        SmartDashboard.putNumber("Current kD", kD);
    }

    public void getPIDInput(){
        kF = SmartDashboard.getNumber("Input kF", kF); //Second parameter is default value (in this case, current value)
        kP = SmartDashboard.getNumber("Input kP", kP);
        kI = SmartDashboard.getNumber("Input kI", kI);
        kD = SmartDashboard.getNumber("Input kD", kD);
        startMotors();
    }

}
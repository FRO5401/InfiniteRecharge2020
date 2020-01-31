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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

public class DriveBase extends Subsystem {
    // Motors
    private VictorSPX lDrive1;
    private VictorSPX rDrive1;
    private VictorSPX lDrive2;
    private VictorSPX rDrive2;
    private TalonSRX lDrive3;
    private TalonSRX rDrive3;

    // Solenoids
    private Solenoid gearShifter;

    // Sensors
    private Encoder lEncoder;
    private Encoder rEncoder;
    private AHRS navxGyro;
 

 public DriveBase() {
    // Instantiate Motors
    lDrive1 = new VictorSPX(RobotMap.DRIVE_MOTOR_L1);
    rDrive1 = new VictorSPX(RobotMap.DRIVE_MOTOR_R1);
    lDrive2 = new VictorSPX(RobotMap.DRIVE_MOTOR_L2);
    rDrive2 = new VictorSPX(RobotMap.DRIVE_MOTOR_R2);
    lDrive3 = new TalonSRX(RobotMap.DRIVE_MOTOR_L3);
    rDrive3 = new TalonSRX(RobotMap.DRIVE_MOTOR_R3);

    // Instantiate Solenoid.
    gearShifter = new Solenoid(RobotMap.GEAR_SHIFTER);

    // Instantiate Sensors
    navxGyro = new AHRS(I2C.Port.kMXP);
    lEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
    rEncoder = new Encoder(RobotMap.DRIVE_ENC_RIGHT_A, RobotMap.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);
}

@Override
public void initDefaultCommand() {
    setDefaultCommand(new XboxMove());
}

public void drive(double lDriveDesired, double rDriveDesired) {
    rDriveDesired *= -1;
    // Left inverted in accordance to physical wiring.
    lDrive1.set(ControlMode.PercentOutput, lDriveDesired);
    lDrive2.set(ControlMode.PercentOutput, lDriveDesired);
    lDrive3.set(ControlMode.PercentOutput, lDriveDesired);
    rDrive1.set(ControlMode.PercentOutput, rDriveDesired);
    rDrive2.set(ControlMode.PercentOutput, rDriveDesired);
    rDrive3.set(ControlMode.PercentOutput, rDriveDesired);
}

public void stopMotors() {
    lDrive1.set(ControlMode.PercentOutput, 0);
    lDrive2.set(ControlMode.PercentOutput, 0);
    rDrive3.set(ControlMode.PercentOutput, 0);
    rDrive1.set(ControlMode.PercentOutput, 0);
    rDrive2.set(ControlMode.PercentOutput, 0);
    rDrive3.set(ControlMode.PercentOutput, 0);
}

public void shiftLowtoHigh() {
    gearShifter.set(false);
    setDPPHighGear();
}

public void shiftHightoLow() {
    gearShifter.set(true);
    setDPPLowGear();
}

public void setDPPHighGear() {
    lEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_LEFT_DPP);
    rEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_RIGHT_DPP);
}

public void setDPPLowGear() {
    lEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_LEFT_DPP);
    rEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_RIGHT_DPP);
}

public double getEncoderDistance(int encoderNumber) {
    double lDistAdj = lEncoder.getDistance();
    double rDistAdj = rEncoder.getDistance();
    double avgDistance = (lDistAdj + rDistAdj) / 2;

    if (encoderNumber == 1) {
      return lDistAdj;
    } else if (encoderNumber == 2) {
      return rDistAdj;
    } else {
      return avgDistance;
    }
}

public double getGyroAngle(){
    double angle = navxGyro.getAngle();
    return angle;
}

public double getGyroPitch(){
    double pitch = navxGyro.getPitch();
    return pitch;
}

// Reports all information from drivebase to SmartDashboard
public void reportDriveBaseSensors() {
    // Misc.
    SmartDashboard.putBoolean("NavX Connection", navxGyro.isConnected());
    SmartDashboard.putBoolean("DriveBase Current Gear", gearShifter.get());
    // Encoders
    SmartDashboard.putNumber("Left Enc Raw", lEncoder.get());
    SmartDashboard.putNumber("Right Enc Raw", rEncoder.get());
    SmartDashboard.putNumber("Left Enc Adj", lEncoder.getDistance());
    SmartDashboard.putNumber("Right Enc Adj", rEncoder.getDistance());
    // NavX
    SmartDashboard.putNumber("NaxX Angle", navxGyro.getAngle());
    SmartDashboard.putNumber("NavX Pitch", navxGyro.getPitch());
    SmartDashboard.putNumber("NavX Yaw", navxGyro.getYaw());
    // Victors
    SmartDashboard.putNumber("Left VSP1 Speed", lDrive1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left VSP2 Speed", lDrive2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right VSP1", rDrive1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right VSP2", rDrive2.getSelectedSensorVelocity());
}

public void resetEncoders() {
  lEncoder.reset();
  rEncoder.reset();
}

public void resetGyro() {
    navxGyro.reset();
}

}

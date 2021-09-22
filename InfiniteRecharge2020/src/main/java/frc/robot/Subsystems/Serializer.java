package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.RobotMap;


public class Serializer extends SubsystemBase {

    // instanciate the objects
    VictorSP serializerMotor;
    VictorSP kickerMotor;

    // make a constructor and declare the variables
    public Serializer() {

        serializerMotor = new VictorSP(RobotMap.SERIALIZER_MOTOR); 
        kickerMotor = new VictorSP(RobotMap.KICKER_MOTOR); 
    }

    public void runSerializer(String choice) {
        double serializerSpeed;
        if (choice.equals("OUT")) {
            serializerSpeed = -0.8;
        }
        else if (choice.equals("IN")) {
            serializerSpeed = 0.8;
        }
        else {
            serializerSpeed = 0;
        }
        serializerMotor.set(serializerSpeed); // Change back to velocity after testing master/slave
        //Make first velocity 1
    }

    public double getSerializerVelocity() {
        return serializerMotor.getSpeed();
    }

    public void runKicker(String choice) {
        double kickerSpeed;
        if (choice.equals("OUT")) {
            kickerSpeed = -0.8;
        }
        else if (choice.equals("IN")) {
            kickerSpeed = 0.8;
        }
        else {
            kickerSpeed = 0;
        }
        kickerMotor.set(kickerSpeed); // Change back to velocity after testing master/slave
        //Make first velocity 1
    }

    public double getKickerVelocity() {
        return kickerMotor.getSpeed();
    }    

    public void reportValues(){
        SmartDashboard.putNumber("Serializer Speed", getSerializerVelocity());
        SmartDashboard.putNumber("Kicker Speed", getKickerVelocity());
    }

}
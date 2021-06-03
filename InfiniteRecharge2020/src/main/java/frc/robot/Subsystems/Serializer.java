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


public class Serializer extends Subsystem {

    // instanciate the objects
    VictorSP serializerMotor;
    VictorSP kickerMotor;

    // make a constructor and declare the variables
    public Serializer() {

        serializerMotor = new VictorSP(RobotMap.SERIALIZER_MOTOR); 
        kickerMotor = new VictorSP(RobotMap.KICKER_MOTOR); 
    }

    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new Kicker()); NO IDEA WHAT TO PUT HERE 

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
        serializerMotor.set(ControlMode.PercentOutput, serializerSpeed); //Change back to velocity after testing master/slave
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
        kickerMotor.set(ControlMode.PercentOutput, kickerSpeed); //Change back to velocity after testing master/slave
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
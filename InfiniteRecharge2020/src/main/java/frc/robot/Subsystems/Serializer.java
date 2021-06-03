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
    VictorSP beltMotor;

    // make a constructor and declare the variables
    public Serializer() {

        serializerMotor = new VictorSP(RobotMap.SERIALIZER_MOTOR); 
        beltMotor = new VictorSP(RobotMap.BELT_MOTOR); 
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

    public void runBelt(String choice) {
        double beltSpeed;
        if (choice.equals("OUT")) {
            beltSpeed = -0.8;
        }
        else if (choice.equals("IN")) {
            beltSpeed = 0.8;
        }
        else {
            beltSpeed = 0;
        }
        beltMotor.set(ControlMode.PercentOutput, beltSpeed); //Change back to velocity after testing master/slave
        //Make first velocity 1
    }

    public double getBeltVelocity() {
        return beltMotor.getSpeed();
    }    

    public void reportValues(){
        SmartDashboard.putNumber("Serializer Speed", getSerializerVelocity());
        SmartDashboard.putNumber("Belt Speed", getBeltVelocity());
    }

}
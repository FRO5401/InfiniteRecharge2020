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
    VictorSP serializer;

    // make a constructor and declare the variables
    public Serializer() {

        serializer = new VictorSP(RobotMap.SERIALIZER_MOTOR); 
    }

    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new ShooterMechanism());

    }

    public void startSerializer() {
        serializer.set(ControlMode.PercentOutput, .80); //Change back to velocity after testing master/slave
        //Make first velocity 1
    }

    public void stopSerializer() {
        serializer.set(ControlMode.PercentOutput, 0.0); //Change back to velocity after testing master/slave
    }

    public double getVelocity() {
        return serializer.getSpeed();
    }

    public void reportValues(){
        SmartDashboard.putNumber("Serializer Speed", getVelocity());
    }

}
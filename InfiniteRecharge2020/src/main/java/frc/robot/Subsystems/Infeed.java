package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Commands.XboxMove;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

/**
 * The Infeed is a Subsystem that allows for the intake of lemons.
 */
public class Infeed extends Subsystem {

    TalonSRX lemonController;
    TalonSRX drumMagMover;
    
    DoubleSolenoid armSolenoid1;
    DoubleSolenoid armSolenoid2;
    DoubleSolenoid armSolenoid3;
    DoubleSolenoid armSolenoid4;

    public Infeed() {

        //Hey... when life gives you lemons, squeeze them to juice up those power ports!
        lemonController = new TalonSRX(RobotMap.LEMON_SQUEEZER);

        drumMagMover = new TalonSRX(RobotMap.DRUM_MAG_MOVER);
        armSolenoid1 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_ONE_FORWARD, RobotMap.ARM_SOLENOID_ONE_REVERSE);
        armSolenoid2 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_TWO_FORWARD, RobotMap.ARM_SOLENOID_TWO_REVERSE);
        armSolenoid3 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_THREE_FORWARD, RobotMap.ARM_SOLENOID_THREE_REVERSE);
        armSolenoid4 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_FOUR_FORWARD, RobotMap.ARM_SOLENOID_FOUR_REVERSE);
    }
    
    //Left Trigger, Right Trigger
    public void extend(boolean retracting, boolean extending) {
        if (retracting) {
            armSolenoid4.set(DoubleSolenoid.Value.kReverse);
            armSolenoid3.set(DoubleSolenoid.Value.kReverse);
            armSolenoid2.set(DoubleSolenoid.Value.kReverse);
            armSolenoid1.set(DoubleSolenoid.Value.kReverse);
            SmartDashboard.putBoolean("Extended", true);
        } else if (extending) {
            armSolenoid1.set(DoubleSolenoid.Value.kForward);
            armSolenoid2.set(DoubleSolenoid.Value.kForward);
            armSolenoid3.set(DoubleSolenoid.Value.kForward);
            armSolenoid4.set(DoubleSolenoid.Value.kForward);
            SmartDashboard.putBoolean("Extended", false);
        }
    }
}
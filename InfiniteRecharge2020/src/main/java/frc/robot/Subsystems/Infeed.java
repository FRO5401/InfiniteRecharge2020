package frc.robot.Subsystems

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Commands.XboxMove;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

public class Infeed extends Subsystem {

    TalonSRX lemonController;
    TalonSRX drumMagMover;
    
    DoubleSolenoid armSolenoid1;
    DoubleSolenoid armSolenoid2;
    DoubleSolenoid armSolenoid3;
    DoubleSolenoid armSolenoid4;

    public Infeed() {
        lemonController = new TalonSRX(RobotMap.LEMON_SQUEEZER);
        drumMagMover = new TalonSRX(RobotMap.DRUM_MAG_MOVER);
        armSolenoid1 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_ONE);
        armSolenoid2 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_TWO);
        armSolenoid3 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_THREE);
        armSolenoid4 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_FOUR);
    }
    
    public void extend(boolean retracting, boolean extending) { //Left Trigger, Right Trigger
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
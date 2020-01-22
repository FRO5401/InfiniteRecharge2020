package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Commands.InfeedControl;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * The Infeed is a Subsystem that allows for the intake of lemons.
 */
public class Infeed extends Subsystem {

    private TalonSRX lemonSqueezer;
    private TalonSRX drumMagMover;

    private DoubleSolenoid armSolenoid1;
    private DoubleSolenoid armSolenoid2;
    private DoubleSolenoid armSolenoid3;
    private DoubleSolenoid armSolenoid4;

    public Infeed() {
        //Hey... when life gives you lemons, squeeze them to juice up those power ports!
        lemonSqueezer = new TalonSRX(RobotMap.LEMON_SQUEEZER);

        //And move those drum magazines to squeeze even more!
        drumMagMover = new TalonSRX(RobotMap.DRUM_MAG_MOVER);

        //Plus, you can never run out of those lemons if you can grab them using arms.
        armSolenoid1 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_ONE_FORWARD, RobotMap.ARM_SOLENOID_ONE_REVERSE);
        armSolenoid2 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_TWO_FORWARD, RobotMap.ARM_SOLENOID_TWO_REVERSE);
        armSolenoid3 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_THREE_FORWARD, RobotMap.ARM_SOLENOID_THREE_REVERSE);
        armSolenoid4 = new DoubleSolenoid(RobotMap.ARM_SOLENOID_FOUR_FORWARD, RobotMap.ARM_SOLENOID_FOUR_REVERSE);
    }

    //Left Trigger, Right Trigger - Teleop
    public void extend(boolean retracting, boolean extending) {
        if (retracting) {
            //FRA - For the Retraction of Arms
            armSolenoid4.set(DoubleSolenoid.Value.kReverse);
            armSolenoid3.set(DoubleSolenoid.Value.kReverse);
            armSolenoid2.set(DoubleSolenoid.Value.kReverse);
            armSolenoid1.set(DoubleSolenoid.Value.kReverse);
            SmartDashboard.putBoolean("Extended", false);
        } else if (extending) {
            //FEA - For the Extension of Arms
            armSolenoid1.set(DoubleSolenoid.Value.kForward);
            armSolenoid2.set(DoubleSolenoid.Value.kForward);
            armSolenoid3.set(DoubleSolenoid.Value.kForward);
            armSolenoid4.set(DoubleSolenoid.Value.kForward);
            SmartDashboard.putBoolean("Extended", true);
        }
    }

    public void moveToInfeedPosition() {
        //Fake variables and methods for unknown drum mag subsystem - DNA (Do Not Activate)
        drumMagMover.move(drumMagAngle);
        lemonSqueezer.move(drumMagAngle);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new InfeedControl());
    }

    //Autonomous Infeed Direction
	public void infeedDirection(int infeedIn) {
        if (infeedIn == -1) {
            //Left Trigger Pressed
            armSolenoid4.set(DoubleSolenoid.Value.kReverse);
            armSolenoid3.set(DoubleSolenoid.Value.kReverse);
            armSolenoid2.set(DoubleSolenoid.Value.kReverse);
            armSolenoid1.set(DoubleSolenoid.Value.kReverse);
            SmartDashboard.putBoolean("Extended", false);
        } else if (infeedIn == 1) {
            //Right Trigger Pressed
            armSolenoid1.set(DoubleSolenoid.Value.kForward);
            armSolenoid2.set(DoubleSolenoid.Value.kForward);
            armSolenoid3.set(DoubleSolenoid.Value.kForward);
            armSolenoid4.set(DoubleSolenoid.Value.kForward);
            SmartDashboard.putBoolean("Extended", true);
        }
	}
}
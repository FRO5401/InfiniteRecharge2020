
package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;

public class InfeedControl extends Command {
    private int inOrOut;

    public FeederControl() {
        requires(Robot.infeed);
    }
}
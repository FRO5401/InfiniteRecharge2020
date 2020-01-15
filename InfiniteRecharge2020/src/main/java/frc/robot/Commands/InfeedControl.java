
package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class InfeedControl extends Command {
    private int inOrOut;

    public InfeedControl() {
        requires(Robot.infeed);

    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
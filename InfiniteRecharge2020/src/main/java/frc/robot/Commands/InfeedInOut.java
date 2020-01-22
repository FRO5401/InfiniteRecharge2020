package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

//This command is for autonomous only.
public class InfeedInOut extends Command {
    
    private int infeedIn;

    public InfeedInOut(int direction) {
        requires(Robot.infeed);
        infeedIn = direction;
    }

    protected void initialize() {
        Robot.infeed.infeedDirection(infeedIn);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    protected void execute() {
    }
}
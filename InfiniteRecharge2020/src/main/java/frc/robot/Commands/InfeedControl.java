
package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

//This command is for teleop only.
public class InfeedControl extends Command {
    private int inOrOut;

    public InfeedControl() {
        requires(Robot.infeed);

        inOrOut = 0;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    protected void execute() {
        inOrOut = Robot.oi.getXboxTriggers_Operator();
        boolean retracting = false;
        boolean extending = false;
        //Left Trigger Pressed
        if (inOrOut == -1) {
            retracting = true;
        } else if (inOrOut == 1) {
            //Right Trigger Pressed
            extending = true;
        }

        Robot.infeed.extend(retracting, extending);
    }

    
}
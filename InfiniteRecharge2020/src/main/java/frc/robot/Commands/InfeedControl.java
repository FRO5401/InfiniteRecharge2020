package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.RobotMap;
import frc.robot.Subsystems.Infeed;

public class InfeedControl extends CommandBase{
    
    boolean infeedIn;
    boolean infeedOut;
    boolean changeDeployStatus;
    boolean doneDeploying;
    Infeed infeed;
    Controls controls;

    public InfeedControl(Infeed m_infeed, Controls m_controls) {
        controls = m_controls;
        infeed = m_infeed;
        addRequirements(infeed);
    }

    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        infeedIn = controls.xboxButton(controls.xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
        infeedOut = controls.xboxButton(controls.xboxOperator, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
        changeDeployStatus = controls.xboxButton(controls.xboxOperator, RobotMap.XBOX_BUTTON_B);
        doneDeploying = false; //Needs to be false for initial deploy

    //Infeed Control
    if(infeedIn){
      infeed.runInfeed("IN");
    }
    else if(infeedOut){
      infeed.runInfeed("OUT");
    }
    else{
      infeed.runInfeed("STOP");
    }

    //Deploy control
    if(changeDeployStatus && doneDeploying == false){
      if(infeed.getDeployStatus() && doneDeploying == false){
        infeed.deployInfeed(false);
      }
      else if(!infeed.getDeployStatus() && doneDeploying == false){
        infeed.deployInfeed(true);
      }
      doneDeploying = true;
    }

    if(!changeDeployStatus){
      doneDeploying = false;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

}   

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.Serializer;
import frc.robot.Subsystems.Shooter;

public class ShooterMechanism extends CommandBase {
  /**
   * Creates a new ShooterMechanism.
   */
  boolean controlShooter = false;
  boolean readyShooter;
  boolean shooterRunning = false;
  private final Shooter shooter;
  private final Controls controls;
  private final Serializer serializer;

  //For PID testing
  public int dPad;


  public ShooterMechanism(Shooter m_shooter, Controls m_controls, Serializer m_serializer) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
    controls = m_controls;
    serializer = m_serializer;
    addRequirements(shooter);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Maybe put a stop here later idk
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //NOTE: We have to put code for expelling the balls from the shooter in case of a jam

    shooter.reportValues();
    controlShooter = controls.xboxButton(controls.xboxOperator,RobotMap.XBOX_BUTTON_X);
    readyShooter = controls.xboxButton(controls.xboxOperator,RobotMap.XBOX_BUTTON_Y);

    //For Pid testing
    dPad = controls.xboxDPad(controls.xboxOperator);

    //For PID testing
    if(dPad == 180){
      shooter.getPIDInput();
    }

    if(readyShooter) {
      shooterRunning = !shooterRunning;
    }

    if(shooterRunning) {
      shooter.startMotors();
    }
    else {
      shooter.stopMotors();
    }

    if(controlShooter == true && (shooter.getVelocity() > 500)) {
      shooter.stopMotors();
      serializer.runSerializer("STOP");
      serializer.runKicker("STOP");
    }

    if(controlShooter == true && (shooter.getVelocity() < 500)) {
      shooter.runMotors();
      serializer.runSerializer("IN");
      serializer.runKicker("IN");
    }

  }

  // Called once the command ends or is interrupted.
  /*@Override
  public void end() {
    shooter.stopMotors();
    serializer.runSerializer("STOP");
    serializer.runKicker("STOP");
  }

  @Override
  public void interrupted(){
    shooter.stopMotors();
    serializer.runSerializer("STOP");
    serializer.runKicker("STOP");
  }*///FIGURE IT OUT

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Autonomous.*;
import frc.robot.Subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoSelected;
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public static Timer timer;
  public double matchTime;

  public static CompressorSubsystem compressorsubsystem;
  public static NetworkTables networktables;
  public static DriveBase drivebase;
  public static Infeed infeed;
  public static BeltChannel beltchannel;
  public static Shooter shooter;
  public static OI oi;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    chooser.setDefaultOption("Do Nothing", new DoNothing());
    chooser.addOption("Drive Straight", new DriveStraight());
    chooser.addOption("Test Turn", new TestTurn());
    chooser.addOption("Drive Turn Around", new DriveTurnAround());
    chooser.addOption("Shoot Drive Off", new ShootDriveOff());
    chooser.addOption("Shoot Infeed Trench", new ShootInfeedTrench());
    chooser.addOption("Ball Center Test", new BallCenterTest());
    SmartDashboard.putData("Auto choices", chooser);

    compressorsubsystem = new CompressorSubsystem();
    networktables = new NetworkTables();
    drivebase = new DriveBase();
    infeed = new Infeed();
    beltchannel = new BeltChannel();
    shooter = new Shooter();
    timer = new Timer();
    
    oi = new OI();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    matchTime = Timer.getMatchTime();
    SmartDashboard.putNumber("Match Time (sec)", matchTime);

    Robot.drivebase.reportDriveBaseSensors();
    Robot.networktables.updateValue();
    Robot.networktables.reportValues();
    Robot.compressorsubsystem.reportCompressorStatus();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Robot.networktables.resetValues();
    Robot.drivebase.resetSensors();
    autoSelected = chooser.getSelected();
    if(autoSelected != null) {
      autoSelected.start();

    }
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
/*    switch (autoSelected) {
    case DriveStraight:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;    
    }     */
  }

  @Override
  public void teleopInit() {
    Robot.drivebase.resetSensors();
    if (autoSelected != null){
      autoSelected.cancel();
    Robot.networktables.resetValues();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    /*
    if(matchTime >= 10){
      Robot.shooter.runMotors();
    }
    */
    //Robot.drivebase.drive(0.5, 0.5);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
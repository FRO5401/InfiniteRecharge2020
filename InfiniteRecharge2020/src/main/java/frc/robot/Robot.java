/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.*;
import frc.robot.Autonomous.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static DriveBase drivebase;
  public static Infeed infeed;
  public static Shooter shooter;
  public static CompressorSubsystem compressorsubsystem;
  public static Serializer serializer;
  public static NetworkTables networktables;
  public static Turret turret; 

  public static Timer timer;
  public double matchTime;

  public static OI oi;

  private Command autoSelected;
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    chooser.setDefaultOption("Do Nothing", new DoNothing());
    chooser.addOption("Drive Straight", new DriveStraight());
    SmartDashboard.putData("Auto choices", chooser);

    matchTime = Timer.getMatchTime();


    drivebase = new DriveBase();
    infeed = new Infeed();
    compressorsubsystem = new CompressorSubsystem();
    //shooter = new Shooter();
    serializer = new Serializer();
    networktables = new NetworkTables();
    turret = new Turret();
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
    //Robot.networktables.reportValues();
    //Robot.turret.reportTurretInfeedSensors();
    //Robot.shooter.reportValues();
    Robot.serializer.reportValues();
    Robot.infeed.reportValues();
    Robot.compressorsubsystem.reportCompressorStatus();
    Robot.turret.reportTurretInfeedSensors();
    //Robot.networktables.updateValue();
    //Robot.drivebase.visionMove();

    SmartDashboard.putNumber("Match Time (sec)", matchTime);
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
//    Robot.drivebase.resetEncoders();
    autoSelected = chooser.getSelected();
    if(autoSelected != null) {
      autoSelected.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      Scheduler.getInstance().run();
      if(matchTime < 7 & matchTime > -1){
        //Robot.shooter.runMotors();
      }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

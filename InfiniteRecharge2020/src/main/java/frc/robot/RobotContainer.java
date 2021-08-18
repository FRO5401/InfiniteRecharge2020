package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Autonomous.SetTrajectoryPath;
import frc.robot.Commands.XboxMove;
import frc.robot.Subsystems.DriveBase;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {


    // The robot's subsystems
    private final DriveBase drivebase = new DriveBase();
    private final Controls controls = new Controls();


    /////////////////////////////OI//////////////////////////////////
  
  


    ///////////////ROBOT CONTAINER STUFF///////////////////////




    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Configure the button bindings
      configureButtonBindings();
      


      // Configure default commands
      // Set the default drive command to split-stick arcade drive
      drivebase.setDefaultCommand(new XboxMove(drivebase, controls));
          //CORRECTED FOR CURRENT CONTROLS, MIGHT WORK ABOUT A 50% not gonna lie);
    }
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
      // Drive at half speed when the right bumper is held
      controls.xboxRightBumper_Driver
      .whenPressed(() -> drivebase.setMaxOutput(0.5))
      .whenReleased(() -> drivebase.setMaxOutput(1));
    
      
    }
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // Create a voltage constraint to ensure we don't accelerate too fast
      var autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(
                  RobotMap.ksVolts,
                  RobotMap.kvVoltSecondsPerMeter,
                  RobotMap.kaVoltSecondsSquaredPerMeter),
              RobotMap.kDriveKinematics,
              10);
  
      // Create config for trajectory
      TrajectoryConfig config =
          new TrajectoryConfig(
                  RobotMap.kMaxSpeedMetersPerSecond,
                  RobotMap.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(RobotMap.kDriveKinematics)
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint);
  
      // An example trajectory to follow.  All units in meters.
      /*Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              // Pass config
              config);*/
  
      
  
      // Run path following command, then stop at the end.
      return new SetTrajectoryPath(drivebase, "paths/DriveStraight.wpilib.json");
    }
  }
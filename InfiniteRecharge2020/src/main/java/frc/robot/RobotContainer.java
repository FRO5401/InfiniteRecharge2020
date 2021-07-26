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
import frc.robot.Subsystems.DriveBase;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {

    //Trajectory Stuff for later
    /*
    String trajectoryJSON = "paths/YourPath.wpilib.json";
    Trajectory trajectory = new Trajectory();*/

    // The robot's subsystems
    private final DriveBase drivebase = new DriveBase();


    /////////////////////////////OI//////////////////////////////////
  
    // The driver's controller
    public XboxController xboxDriver = new XboxController(RobotMap.XBOX_CONTROLLER_DRIVER);
    public XboxController xboxOperator = new XboxController(RobotMap.XBOX_CONTROLLER_OPERATOR);
  
    //Buttons (Driver)
    JoystickButton xboxA_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_A);
    JoystickButton xboxB_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_B);
    JoystickButton xboxX_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_X);
    JoystickButton xboxY_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_Y);
    JoystickButton xboxLeftBumper_Driver  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    JoystickButton xboxRightBumper_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
    JoystickButton xboxBack_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_BACK);
    JoystickButton xboxStart_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_START);
    JoystickButton xboxL3_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_L3);
    JoystickButton xboxR3_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_R3);
    
      //Buttons (Operator)
    JoystickButton xboxA_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_A);
    JoystickButton xboxB_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_B);
    JoystickButton xboxX_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_X);
    JoystickButton xboxY_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_Y);
    JoystickButton xboxLeftBumper_Operator  = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    JoystickButton xboxRightBumper_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
    JoystickButton xboxBack_Operator		= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_BACK);
    JoystickButton xboxStart_Operator		= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_START);
    JoystickButton xboxL3_Operator		  	= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_L3);
    JoystickButton xboxR3_Operator		  	= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_R3);
  
    public double xboxAxis(Joystick xboxController, int xboxAxis){
      return xboxController.getRawAxis(xboxAxis);
    }
  
    public boolean xboxButton(Joystick xboxController, int xboxButton){
      return xboxController.getRawButton(xboxButton);
    }
  
    public int xboxAxisAsButton(Joystick xboxController, int xboxAxis){
      if(xboxController.getRawAxis(xboxAxis) > RobotMap.AXIS_THRESHOLD){
        return 1;
      }
      else if(xboxController.getRawAxis(xboxAxis) < (-1 * RobotMap.AXIS_THRESHOLD)){
        return -1;
      }
      else{
        return 0;
      }
    }
  
    public int xboxDPad(Joystick xboxController){
      return xboxController.getPOV();
    }
  


    ///////////////ROBOT CONTAINER STUFF///////////////////////




    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Configure the button bindings
      configureButtonBindings();


      //For later, when actually using pathweaver
      /*
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
       } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
      */


      // Configure default commands
      // Set the default drive command to split-stick arcade drive
      drivebase.setDefaultCommand(
          // A split-stick arcade command, with forward/backward controlled by the left
          // hand, and turning controlled by the right.
          new RunCommand(
              () ->
                  drivebase.arcadeDrive(
                      xboxDriver.getTriggerAxis(GenericHID.Hand.kRight),
                      xboxDriver.getX(GenericHID.Hand.kRight)),
              drivebase));
    }
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
      // Drive at half speed when the right bumper is held
      xboxRightBumper_Driver
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
      Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              // Pass config
              config);
  
      RamseteCommand ramseteCommand =
          new RamseteCommand(
              exampleTrajectory,
              drivebase::getPose,
              new RamseteController(RobotMap.kRamseteB, RobotMap.kRamseteZeta),
              new SimpleMotorFeedforward(
                  RobotMap.ksVolts,
                  RobotMap.kvVoltSecondsPerMeter,
                  RobotMap.kaVoltSecondsSquaredPerMeter),
              RobotMap.kDriveKinematics,
              drivebase::getWheelSpeeds,
              new PIDController(RobotMap.kPDriveVel, 0, 0),
              new PIDController(RobotMap.kPDriveVel, 0, 0),
              // RamseteCommand passes volts to the callback
              drivebase::tankDriveVolts,
              drivebase);
  
      // Reset odometry to the starting pose of the trajectory.
      drivebase.resetOdometry(exampleTrajectory.getInitialPose());
  
      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> drivebase.tankDriveVolts(0, 0));
    }
  }
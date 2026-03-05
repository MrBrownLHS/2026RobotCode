// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
// WPILib imports for controller handling, commands, dashboards and utilities
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.commands.LaunchCloseCommand;
import frc.robot.commands.LaunchFarCommand;
import frc.robot.commands.StopAllCommand;
import frc.robot.commands.CollectCommand;
import frc.robot.commands.AutoCenterLaunchClimb;
// Project commands and subsystems
import frc.robot.commands.SwerveController;

//import frc.robot.commands.AutoFuelLaunch;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.SuperSystem;
import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;
import frc.robot.utilities.DriverHUD;


public class RobotContainer {
  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;
  private final JoystickButton slowDriveMode;
  
  // Co-pilot controller (uses Command-based helpers)
  //private final Joystick CoPilotController = new Joystick(1);
  //private final XboxController CoPilotController = new XboxController(1);
  private static final CommandXboxController CopilotCommandController = new CommandXboxController(1);

  // Subsystem instances created once and shared across commands
  
  private final Climber climber = new Climber();
  private final Kicker kicker = new Kicker();
  private final Agitator agitator = new Agitator();
  private final Intake intake = new Intake();
  private final Launcher launch = new Launcher();
  private final Hopper hopper = new Hopper();

  // SuperSystem coordinates states across these shared subsystems
  private final SuperSystem superSystem = new SuperSystem(launch, kicker, intake, hopper, agitator);

  private final LaunchFarCommand launchFarCommand = new LaunchFarCommand(superSystem);
  private final LaunchCloseCommand launchCloseCommand = new LaunchCloseCommand(superSystem);
  private final StopAllCommand stopAllCommand = new StopAllCommand(superSystem);
  private final CollectCommand collectCommand = new CollectCommand(superSystem);

   

  // Controller axis mappings (populated from Constants)
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;
  
  // Driver joystick and the primary swerve subsystem
  private final Joystick DriverController= new Joystick(0);
  private final Swerve swerveSubsystem = new Swerve();

  // Dashboard chooser for autonomous selection
  private final SendableChooser<Command> autoChooser;

  // Slew rate limiters to smooth joystick inputs for translation/rotation
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);


  public RobotContainer() {
   
    // Start data logging and put dashboard widgets for autonomous selection
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Robot Initialized");
        
    autoChooser = new SendableChooser<>();
    // Populate autonomous chooser (default safe/no-op + Auto Center Launch Climb)
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Auto Center Launch Climb", new AutoCenterLaunchClimb(swerveSubsystem, launch, climber));
    SmartDashboard.putData("Auto Mode", autoChooser);


    UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(640, 480);
        camera.setFPS(30);
      
    
        
    // Map driver buttons to JoystickButton wrappers using constants
    resetHeading = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kY.value);
    robotCentric = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kX.value);
    slowDriveMode = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kRightBumper.value);
    
    // Read axis numbers from constants so we can use getRawAxis(index)
    translationAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftY.value;
    strafeAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftX.value;
    rotationAxis = Constants.ControllerRawButtons.XboxController.Axis.kRightX.value;

    // Set the default swerve drive command that reads joystick axes and
    // sends velocities to the swerve subsystem while no other command runs it.
    swerveSubsystem.setDefaultCommand(new SwerveController(
            swerveSubsystem,
            () -> -translationLimiter.calculate(DriverController.getRawAxis(translationAxis) * 0.50),
            () -> -strafeLimiter.calculate(DriverController.getRawAxis(strafeAxis) * 0.50),
            () -> rotationLimiter.calculate(DriverController.getRawAxis(rotationAxis) * 0.50),
            () -> robotCentric.getAsBoolean()) // lambda probably not needed but why not
    );


    

    // Configure button bindings for control mappings
    configureBindings();

    DriverHUD.logSwerve(swerveSubsystem);
    DriverHUD.logLauncher(launch);
    DriverHUD.logSuperSystem(superSystem);
  DriverHUD.logReadyFlags(launch, intake);
    DriverHUD.logClimber(climber);
  }

   

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    
  }
  /**
   * Binds controller buttons to commands. Kept separate for clarity.
   */
  private void configureBindings() {
   
    // Drive Controls: reset heading and a "slow mode" drive command
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    slowDriveMode.whileTrue(new SwerveController(
        swerveSubsystem,
          () -> -translationLimiter.calculate(DriverController.getRawAxis(translationAxis) * 0.25),
          () -> -strafeLimiter.calculate(DriverController.getRawAxis(strafeAxis) * 0.25),
          () -> rotationLimiter.calculate(DriverController.getRawAxis(rotationAxis) * 0.25),
          () -> robotCentric.getAsBoolean())
    );

    // Intake/Index Controls: co-pilot A runs all intake/index/launch collect
    CopilotCommandController.a().whileTrue(launchFarCommand);
    
    CopilotCommandController.b().whileTrue(launchCloseCommand);
    
    CopilotCommandController.x().onTrue(stopAllCommand);

    CopilotCommandController.rightBumper().whileTrue(collectCommand);


    CopilotCommandController.pov(90).whileTrue(
      new RunCommand (
        () -> hopper.setState(
          Hopper.State.EXTENDING), hopper)
    );

    CopilotCommandController.pov(270).whileTrue(
      new RunCommand (
        () -> hopper.setState(
          Hopper.State.RETRACTING), hopper)
    );

      
    // Climber Controls: POV for up/retract and continuous reach commands
    CopilotCommandController.pov(0).onTrue(
    new InstantCommand(
        () -> climber.setState(
          Climber.State.EXTENDING), climber)
    );       

    CopilotCommandController.pov(180).whileTrue(
      new InstantCommand (
        () -> climber.setState(
          Climber.State.CLIMBING), climber)
    );

  }

   public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveController;
import frc.robot.commands.ShortLaunchSequence;
import frc.robot.commands.FarLaunchSequence;
import frc.robot.commands.AutoFuelLaunch;
import frc.robot.commands.CollectFuel;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ClimberLift;
import frc.robot.subsystems.ClimberLong;
import frc.robot.subsystems.ClimberShort;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launch;
import frc.robot.utilities.Constants;






public class RobotContainer {
  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;
  private final JoystickButton slowDriveMode;
  
  //private final Joystick CoPilotController = new Joystick(1);
  //private final XboxController CoPilotController = new XboxController(1);
  private static final CommandXboxController CopilotCommandController = new CommandXboxController(1);

  private final ClimberLift climberLift = new ClimberLift();
  private final ClimberLong climberLong = new ClimberLong();
  private final ClimberShort climberShort = new ClimberShort();
  private final Index index = new Index();
  private final Intake intake = new Intake();
  private final Launch launch = new Launch();
  
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;
  
  private final Joystick DriverController= new Joystick(0);
  private final Swerve swerveSubsystem = new Swerve();

  private final SendableChooser<Command> autoChooser;

  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);


  public RobotContainer() {
   
    DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.log("Robot Initialized");
        
     
    autoChooser = new SendableChooser<>();
        
        SmartDashboard.putData("Auto Mode", autoChooser);
        
    
    resetHeading = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kY.value);
    robotCentric = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kX.value);
    slowDriveMode = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kRightBumper.value);
    

    translationAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftY.value;
    strafeAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftX.value;
    rotationAxis = Constants.ControllerRawButtons.XboxController.Axis.kRightX.value;


      
    swerveSubsystem.setDefaultCommand(new SwerveController(
            swerveSubsystem,
            () -> -translationLimiter.calculate(DriverController.getRawAxis(translationAxis) * 0.50),
            () -> -strafeLimiter.calculate(DriverController.getRawAxis(strafeAxis) * 0.50),
            () -> rotationLimiter.calculate(DriverController.getRawAxis(rotationAxis) * 0.50),
            () -> robotCentric.getAsBoolean()) // lambda probably not needed but why not
    );

    climberLift.setDefaultCommand(climberLift.ClimberLiftStop());
    intake.setDefaultCommand(intake.IntakeStop());
    index.setDefaultCommand(index.IndexStop());
    launch.setDefaultCommand(launch.LaunchStop());

   
  configureBindings(); 
  }

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    
  }
  private void configureBindings() {
   
  //Drive Controls
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    slowDriveMode.whileTrue(new SwerveController(
        swerveSubsystem,
          () -> -translationLimiter.calculate(DriverController.getRawAxis(translationAxis) * 0.25),
          () -> -strafeLimiter.calculate(DriverController.getRawAxis(strafeAxis) * 0.25),
          () -> rotationLimiter.calculate(DriverController.getRawAxis(rotationAxis) * 0.25),
          () -> robotCentric.getAsBoolean())
    );

  //Intake/Index Controls
    CopilotCommandController.a().whileTrue(new CollectFuel(index, intake, launch));
    CopilotCommandController.b().whileTrue(intake.IntakeReverse());
    CopilotCommandController.x().whileTrue(index.IndexReverse());

  //Launch Controls
    CopilotCommandController.rightBumper().whileTrue(new FarLaunchSequence()); //Commands Not Written Yet
    CopilotCommandController.leftBumper().whileTrue(new ShortLaunchSequence()); //Commands Not Written Yet

  //Climber Controls
    CopilotCommandController.pov(0).whileTrue(climberLift.ClimberLiftUp());
    CopilotCommandController.pov(180).whileTrue(climberLift.ClimberLiftRetract());
    climberLong.setDefaultCommand(climberLong.LongClimberReach(() -> CopilotCommandController.getRightY()));
    climberShort.setDefaultCommand(climberShort.ShortClimberReach(() -> CopilotCommandController.getLeftY()));

    
  }

   public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Subsystem base and command helpers
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// REV SparkMax motor controller classes and config helpers
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Project constants for IDs and tuning
import frc.robot.utilities.Constants;


/**
 * Subsystem representing the climber-lift motor. Provides simple factory
 * commands to raise, retract, and stop the lift. Motor configuration is
 * applied during construction.
 */
public class ClimberLift extends SubsystemBase {
  // Motor that actuates the climber lift
  private final SparkMax m_ClimberLift;
  // Configuration object used to set current limits, idle mode, etc.
  private SparkMaxConfig climberLiftMotorConfig;
 
  /**
   * Create a new ClimberLift subsystem and configure its motor.
   */
  public ClimberLift() {
    // Instantiate the motor with the ID from Constants and brushless type
    m_ClimberLift = new SparkMax(Constants.ClimberConstants.CLIMBER_LIFT_MOTOR_ID, MotorType.kBrushless);

    // Prepare and apply motor configuration
    climberLiftMotorConfig = new SparkMaxConfig();
    configureIntakeMotor(m_ClimberLift, climberLiftMotorConfig);
  }

  /**
   * Configure motor defaults (idle mode, current limits, voltage compensation).
   * This keeps hardware setup centralized and easy to tune.
   */
  private void configureIntakeMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
  }

  /**
   * Factory command: run the lift up at a fixed percent output while scheduled.
   * The returned RunCommand already declares this subsystem as a requirement.
   */
  public RunCommand ClimberLiftUp() {
    return new RunCommand(() -> {
      m_ClimberLift.set(0.5);
    }, this);
  }

  /**
   * Factory command: retract the lift (run motor in reverse) while scheduled.
   */
  public RunCommand ClimberLiftRetract() {
    return new RunCommand(() -> {
      m_ClimberLift.set(-0.5);
    }, this);
  }

  /**
   * Factory command: stop the lift motor immediately (InstantCommand).
   */
  public Command ClimberLiftStop() {
    return new InstantCommand(() -> {
      m_ClimberLift.set(0.0);
    }, this);
  }

  @Override
  public void periodic() {
    // Periodic hook - runs every scheduler cycle. Left empty because this
    // simple subsystem does not need regular background work.
  }
}

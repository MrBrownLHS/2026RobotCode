// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPILib command and utility imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

// REV motor controller imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Project constants and functional interfaces
import frc.robot.utilities.Constants;
import java.util.function.DoubleSupplier;


/**
 * Subsystem for the long climber actuator. Provides factory commands to
 * control the long-stage climber using a joystick input or stop instantly.
 */
public class ClimberLong extends SubsystemBase {
  // Motor for the long-stage climber
  private SparkMax m_ClimberLong;
  // Configuration holder for motor tuning (current limits, idle mode, etc.)
  private final SparkMaxConfig climberMotorConfig;
  // Slew rate limiter to smooth joystick inputs and prevent sudden jerks
  private final SlewRateLimiter climberRateLimiter;

  /**
   * Create the ClimberLong subsystem, configure hardware, and prepare helpers.
   */
  public ClimberLong() {
    climberRateLimiter = new SlewRateLimiter(Constants.ClimberConstants.CLIMBER_RATE_LIMIT);
    m_ClimberLong = new SparkMax(Constants.ClimberConstants.CLIMBER_LONG_MOTOR_ID, MotorType.kBrushless);
    climberMotorConfig = new SparkMaxConfig();

    // Apply motor configuration (idle mode, current limits, voltage comp)
    configureClimberLongMotor(m_ClimberLong, climberMotorConfig);
  }

  /**
   * Apply standard motor configuration for the climber motor.
   */
  private void configureClimberLongMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
  }

  /**
   * Continuous command that reads a joystick input supplier and drives the
   * climber motor. Applies a deadband, slew-rate limiting, and scales the
   * final output so the climber moves at a safe speed.
   *
   * @param joystickInput supplier providing the joystick axis value
   */
  public Command LongClimberReach (DoubleSupplier joystickInput) {
    return new RunCommand(() -> {
      double rawInput = joystickInput.getAsDouble();
      double adjustedInput = (Math.abs(rawInput) > Constants.ClimberConstants.CLIMBER_DEADBAND) ? rawInput : 0.0;
      double limitedInput = climberRateLimiter.calculate(adjustedInput * 0.30);
      m_ClimberLong.set(limitedInput);
    }, this);
  }

  // Example of a commented-out autonomous helper for running the motor at a
  // fixed speed for a timeout. Left in place for future use.
  // public Command ClimberLongAuto(double speed) {
  //   return new RunCommand(() -> {
  //           m_ClimberLong.set(speed);
  //       }, this).withTimeout(5.0
  //   );
  // }

  /**
   * Instant command to stop the climber motor immediately.
   */
  public Command LongClimberStop() {
    return new InstantCommand(() -> {
      m_ClimberLong.set(0.0);
    }, this);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.DoubleSupplier;


public class ClimberShort extends SubsystemBase {
  private SparkMax m_ClimberShort;
  private final SparkMaxConfig climberMotorConfig;
  private final SlewRateLimiter climberRateLimiter;

  public ClimberShort() {
    climberRateLimiter = new SlewRateLimiter(Constants.ClimberConstants.CLIMBER_RATE_LIMIT);
    m_ClimberShort = new SparkMax(Constants.ClimberConstants.CLIMBER_SHORT_MOTOR_ID, MotorType.kBrushless);
    climberMotorConfig = new SparkMaxConfig();

    configureClimberShortMotor(m_ClimberShort, climberMotorConfig);
  }

  private void configureClimberShortMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
  }

  public Command ShortClimberReach (DoubleSupplier joystickInput) {
    return new RunCommand(() -> {
      double rawInput = joystickInput.getAsDouble();
      double adjustedInput = (Math.abs(rawInput) > Constants.ClimberConstants.CLIMBER_DEADBAND) ? rawInput : 0.0;
      double limitedInput = climberRateLimiter.calculate(adjustedInput * 0.30);
      m_ClimberShort.set(limitedInput);
    }, this);
  }

  // public Command ClimberLongAuto(double speed) {
  //   return new RunCommand(() -> {
  //           m_ClimberLong.set(speed);
  //       }, this).withTimeout(5.0
        
  //   );
  // }

  public Command ShortClimberStop() {
    return new InstantCommand(() -> {
      m_ClimberShort.set(0.0);
    }, this);
  }
}
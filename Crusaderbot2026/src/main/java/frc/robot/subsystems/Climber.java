// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class Climber extends SubsystemBase {

  public enum State {
    IDLE,
    EXTENDING,
    CLIMBING
  }

  private State currentState = State.IDLE;

  // Single climber motor
  private final SparkMax m_climber;
  private final RelativeEncoder climberEncoder;

  public Climber() {

    m_climber = new SparkMax(
        Constants.ClimberConstants.CLIMBER_WINCH_MOTOR_ID,
        MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

    m_climber.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    climberEncoder = m_climber.getEncoder();

    // Zero position at startup (assumes climber starts retracted)
    climberEncoder.setPosition(0.0);

    m_climber.set(0.0);
  }

  public void setState(State newState) {
    currentState = newState;
  }

  public State getState() {
    return currentState;
  }

  public void stop() {
    m_climber.set(0.0);
  }

  public double getPosition() {
    return climberEncoder.getPosition();
  }

  @Override
  public void periodic() {

    double position = getPosition();

    switch (currentState) {

      case IDLE:
        stop();
        break;

      case EXTENDING:

        if (position >= Constants.ClimberConstants.CLIMBER_MAX_EXTENSION) {
          stop();
          currentState = State.IDLE;
        } else {
          m_climber.set(Constants.ClimberConstants.CLIMBER_EXTEND_SPEED);
        }

        break;

      case CLIMBING:

        if (position <= Constants.ClimberConstants.CLIMBER_MIN_EXTENSION) {
          stop();
          currentState = State.IDLE;
        } else {
          m_climber.set(Constants.ClimberConstants.CLIMBER_RETRACT_SPEED);
        }

        break;
    }

    // ===============================
    // Dashboard Logging
    // ===============================

    Dashboard.logNumber("Climber Position", this::getPosition);
    Dashboard.logString("Climber State", () -> currentState.toString());
  }
}
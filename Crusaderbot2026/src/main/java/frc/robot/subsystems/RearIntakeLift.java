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

public class RearIntakeLift extends SubsystemBase {

  public enum State {
    STORED,
    LIFTED,
    EXTENDED
  }

  private State currentState = State.STORED;

  private final SparkMax m_RearIntakeLift;
  private final RelativeEncoder rearIntakeLiftEncoder;

  // --- Tunable control values ---
  private double kP = 0.02; // Tune this
  private double tolerance = 5.0; // Acceptable error

  public RearIntakeLift() {

    m_RearIntakeLift = new SparkMax(
        Constants.FuelSystemConstants.REAR_INTAKE_LIFT_MOTOR_ID,
        MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

    m_RearIntakeLift.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    rearIntakeLiftEncoder = m_RearIntakeLift.getEncoder();

    // Zero at startup
    rearIntakeLiftEncoder.setPosition(0.0);

    m_RearIntakeLift.set(0.0);
  }

  // -----------------------------
  // STATE CONTROL
  // -----------------------------
  public void setState(State newState) {
    currentState = newState;
  }

  public State getState() {
    return currentState;
  }

  // -----------------------------
  // POSITION LOGIC
  // -----------------------------
  private double getTargetPosition() {
    switch (currentState) {
      case STORED:
        return Constants.FuelSystemConstants.REAR_INTAKE_LIFT_STORED_POSITION;

      case LIFTED:
        return Constants.FuelSystemConstants.REAR_INTAKE_LIFT_LIFTED_POSITION;

      case EXTENDED:
        return Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION;

      default:
        return 0.0;
    }
  }

  public double getPosition() {
    return rearIntakeLiftEncoder.getPosition();
  }

  public void zeroEncoder() {
    rearIntakeLiftEncoder.setPosition(0.0);
  }

  public boolean atSetpoint() {
    return Math.abs(getTargetPosition() - getPosition()) < tolerance;
  }

  public void stop() {
    m_RearIntakeLift.set(0.0);
  }

  // -----------------------------
  // PERIODIC CONTROL LOOP
  // -----------------------------
  @Override
  public void periodic() {

    double target = getTargetPosition();
    double current = getPosition();

    double error = target - current;

    // Simple P control
    double output = kP * error;

    // Clamp output to safe speeds
    output = Math.max(-0.15, Math.min(0.15, output));

    // Stop if within tolerance
    if (Math.abs(error) < tolerance) {
      output = 0.0;
    }

    m_RearIntakeLift.set(output);

    // -----------------------------
    // DASHBOARD OUTPUT (for tuning)
    // -----------------------------
    Dashboard.logNumber("RearIntakeLift Position", () -> getPosition());
    Dashboard.logNumber("RearIntakeLift Target", () -> getTargetPosition());
    Dashboard.logString("RearIntakeLift State", () ->currentState.toString());
  }
}
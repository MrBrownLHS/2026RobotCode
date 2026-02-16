
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Launcher extends SubsystemBase {

  public enum State {
    IDLE,
    LAUNCH_FAR,
    LAUNCH_CLOSE,
    LAUNCH_COLLECT,
    LAUNCH_REVERSE
  }

  private State currentState = State.IDLE;

  private final SparkMax m_Launch;
  private SparkMaxConfig intakeMotorConfig;

  /** Creates a new Launch. */
  public Launcher() {
    m_Launch = new SparkMax(Constants.FuelSystemConstants.LAUNCH_MOTOR_1_ID, MotorType.kBrushless);

    intakeMotorConfig = new SparkMaxConfig();

    configureIntakeMotor(m_Launch, intakeMotorConfig);
  }

  private void configureIntakeMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
  
  }

  public void setState(State newState) {
    currentState = newState;
  }

  public State getState() {
    return currentState;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case IDLE:
        m_Launch.set(0.0);
        break;
      case LAUNCH_FAR:
        m_Launch.set(0.95);
        break;
      case LAUNCH_CLOSE:
        m_Launch.set(0.75);
        break;
      case LAUNCH_COLLECT:
        m_Launch.set(0.25);
        break;
      case LAUNCH_REVERSE:
        m_Launch.set(-0.5);
        break;
    }
  }
}

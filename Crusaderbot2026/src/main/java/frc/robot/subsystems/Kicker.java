
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Kicker extends SubsystemBase {

      public enum State {
        IDLE,
        KICK_COLLECT,
        KICK_CLOSE,
        KICK_FAR,
        KICK_REVERSE
      }
      
  private State currentState = State.IDLE;

  private final SparkMax m_Kicker;
  private SparkMaxConfig kickerMotorConfig;
 
  /** Creates a new Index. */
  public Kicker() {
    m_Kicker = new SparkMax(Constants.FuelSystemConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

    kickerMotorConfig = new SparkMaxConfig();

    configureKickerMotor(m_Kicker, kickerMotorConfig);
  }

  private void configureKickerMotor(SparkMax motor, SparkMaxConfig config){
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
        m_Kicker.set(0.0);
        break;
      case KICK_COLLECT:
        m_Kicker.set(Constants.FuelSystemConstants.KICKER_COLLECT_SPEED);
        break;
      case KICK_CLOSE:
        m_Kicker.set(Constants.FuelSystemConstants.KICKER_CLOSE_SPEED);
        break;
      case KICK_FAR:
        m_Kicker.set(Constants.FuelSystemConstants.KICKER_FAR_SPEED);
        break;
      case KICK_REVERSE:
        m_Kicker.set(-Constants.FuelSystemConstants.KICKER_COLLECT_SPEED);
        break;
      }
    SmartDashboard.putString("Kicker State", currentState.toString());
  }
}
